#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
NTP 服务器延时 / 时钟源稳定性 Benchmark 工具（浏览器界面版）
============================================================
功能：
  - 同时向最多 8 个远程 NTP 服务器发起请求（多线程并发）
  - 测量每个服务器的 往返延时(RTT/delay) 与 时钟偏移(offset)
  - 通过多次采样评估 "时钟源稳定性"(偏移抖动 jitter = 偏移标准差)
  - 以表格 + 图形(折线/柱状) 形式对比 / benchmark
  - 支持导出 CSV

运行：
  python ntp_benchmark.py            # 然后浏览器打开 http://localhost:8000
  python ntp_benchmark.py --port 9000
  python ntp_benchmark.py --host 0.0.0.0 --port 8000   # 允许局域网访问

依赖：仅 Python 标准库（http.server / socket / struct / threading / statistics / csv）。
      无需安装任何第三方包，无需联网。
"""

import socket
import struct
import time
import json
import threading
import statistics
import csv
import io
import webbrowser
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import urlparse

# NTP 时间戳起点 (1900-01-01) 与 Unix 时间戳 (1970-01-01) 的差值(秒)
NTP_EPOCH = 2208988800.0

# 8 个默认服务器
DEFAULT_SERVERS = [
    "192.168.6.44",
    "192.168.6.45",
    "192.168.6.201",
    "cn.ntp.org.cn",
    "edu.ntp.org.cn",
    "pool.ntp.org",
    "cn.pool.ntp.org",
    "time.apple.com",
]

# 8 种用于区分服务器的颜色（与前端保持一致）
COLORS = [
    "#1f77b4", "#ff7f0e", "#2ca02c", "#d62728",
    "#9467bd", "#8c564b", "#e377c2", "#17becf",
]


# ----------------------------------------------------------------------------
# NTP 协议客户端（最小实现，无需 ntplib）
# ----------------------------------------------------------------------------
def _ts_to_float(data: bytes) -> float:
    sec = int.from_bytes(data[:4], "big")
    frac = int.from_bytes(data[4:], "big")
    return sec + frac / 2 ** 32


def _fixed32_to_float(data: bytes) -> float:
    return int.from_bytes(data, "big") / 65536.0


def _ref_id_to_str(data: bytes, stratum: int) -> str:
    if stratum == 1:
        try:
            return data.decode("ascii").strip()
        except Exception:
            return data.hex()
    try:
        return ".".join(str(b) for b in data)
    except Exception:
        return data.hex()


def parse_ntp_response(data: bytes, t0: float, t3: float) -> dict:
    """解析 NTP 响应并计算 offset / delay。
    offset = ((t1 - t0) + (t2 - t3)) / 2   (客户端相对服务器的时钟偏移)
    delay  = (t3 - t0) - (t2 - t1)         (往返路径延时)
    """
    if len(data) < 48:
        raise ValueError("NTP 响应长度不足 48 字节")
    stratum = data[1]
    root_delay = _fixed32_to_float(data[4:8])
    root_disp = _fixed32_to_float(data[8:12])
    ref_id = _ref_id_to_str(data[12:16], stratum)
    t1 = _ts_to_float(data[32:40])
    t2 = _ts_to_float(data[40:48])
    offset = ((t1 - t0) + (t2 - t3)) / 2.0
    delay = (t3 - t0) - (t2 - t1)
    return {
        "stratum": stratum,
        "root_delay": root_delay,
        "root_disp": root_disp,
        "ref_id": ref_id,
        "offset": offset,
        "delay": delay,
    }


def ntp_query(host: str, port: int = 123, timeout: float = 3.0) -> dict:
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.settimeout(timeout)
    try:
        req = bytes([0x1B]) + bytes(47)  # LI=0, VN=3, Mode=3 (客户端)
        t0 = time.time() + NTP_EPOCH
        sock.sendto(req, (host, port))
        data, _addr = sock.recvfrom(48)
        t3 = time.time() + NTP_EPOCH
        return parse_ntp_response(data, t0, t3)
    finally:
        sock.close()


# ----------------------------------------------------------------------------
# 统计
# ----------------------------------------------------------------------------
def compute_stats(samples: list) -> dict:
    ok = [s for s in samples if s.get("ok")]
    n_ok = len(ok)
    n_total = len(samples)
    if n_ok == 0:
        return {
            "ok": 0, "total": n_total, "status": "错误",
            "stratum": "-", "ref": "-",
            "rtt_avg": None, "rtt_min": None, "rtt_max": None, "rtt_std": None,
            "offset_avg": None, "jitter": None, "root_disp": None, "score": None,
        }
    rtts = [s["rtt"] for s in ok]
    offs = [s["offset"] for s in ok]
    rtt_avg = statistics.fmean(rtts)
    rtt_min = min(rtts)
    rtt_max = max(rtts)
    rtt_std = statistics.pstdev(rtts) if n_ok > 1 else 0.0
    offset_avg = statistics.fmean(offs)
    jitter = statistics.pstdev(offs) if n_ok > 1 else 0.0
    # 注意单位：样本入库时（见下方采集循环 "root_disp": r["root_disp"] * 1000.0）
    # 已经把秒换成了毫秒，这里只能取均值，不能再乘 1000 ——
    # 曾经多乘一次，把「根离散」整列放大 1000 倍（1 ms 显示成 1 s 量级）。
    root_disp = statistics.fmean([s.get("root_disp", 0.0) for s in ok])
    score = rtt_avg + jitter  # 越小越好
    return {
        "ok": n_ok, "total": n_total, "status": "完成",
        "stratum": ok[0].get("stratum", "-"),
        "ref": ok[0].get("ref", "-"),
        "rtt_avg": rtt_avg, "rtt_min": rtt_min, "rtt_max": rtt_max, "rtt_std": rtt_std,
        "offset_avg": offset_avg, "jitter": jitter, "root_disp": root_disp, "score": score,
    }


# ----------------------------------------------------------------------------
# Benchmark 控制器（后端状态）
# ----------------------------------------------------------------------------
class Benchmark:
    def __init__(self):
        self.lock = threading.Lock()
        self.stop_event = threading.Event()
        self.running = False
        self.progress = "就绪"
        self.servers = []   # [{host, enabled, samples:[...], status, thread}]

    def start(self, config: dict):
        with self.lock:
            if self.running:
                return False
            self.stop_event.clear()
            self.servers = []
            hosts = config.get("hosts", [])
            enabled = config.get("enabled", [True] * len(hosts))
            for i, h in enumerate(hosts[:8]):
                self.servers.append({
                    "host": (h or "").strip(),
                    "enabled": bool(enabled[i]) if i < len(enabled) else True,
                    "samples": [],
                    "status": "等待",
                    "thread": None,
                })
            if not any(s["host"] for s in self.servers if s["enabled"]):
                self.progress = "请至少填写一个服务器"
                return False
            self.samples_n = max(3, min(60, int(config.get("samples", 12) or 12)))
            self.interval = float(config.get("interval", 1.0) or 1.0)
            self.timeout = float(config.get("timeout", 3.0) or 3.0)
            self.running = True
            self.progress = "运行中…"
            for idx in range(len(self.servers)):
                t = threading.Thread(target=self._worker, args=(idx,), daemon=True)
                self.servers[idx]["thread"] = t
                t.start()
            return True

    def stop(self):
        self.stop_event.set()
        with self.lock:
            self.progress = "正在停止…"

    def reset(self):
        self.stop_event.set()
        with self.lock:
            self.servers = []
            self.running = False
            self.progress = "就绪"

    def _worker(self, idx):
        srv = self.servers[idx]
        host = srv["host"]
        if not host:
            with self.lock:
                srv["status"] = "空"
            return
        n, iv, to = self.samples_n, self.interval, self.timeout
        with self.lock:
            srv["status"] = "查询中"
        for k in range(n):
            if self.stop_event.is_set():
                break
            try:
                r = ntp_query(host, timeout=to)
                with self.lock:
                    srv["samples"].append({
                        "ok": True,
                        "rtt": r["delay"] * 1000.0,
                        "offset": r["offset"] * 1000.0,
                        "stratum": r["stratum"],
                        "ref": r["ref_id"],
                        "root_disp": r["root_disp"] * 1000.0,
                    })
            except Exception as e:
                with self.lock:
                    srv["samples"].append({"ok": False, "err": str(e)[:80]})
            if k < n - 1 and not self.stop_event.is_set():
                time.sleep(iv)
        with self.lock:
            srv["status"] = "已停止" if self.stop_event.is_set() else "完成"
        with self.lock:
            all_done = all(s["status"] in ("完成", "已停止", "空", "错误")
                           for s in self.servers if s["enabled"])
            if all_done and not self.stop_event.is_set() and self.running:
                self.running = False
                self.progress = "完成 · 共 %d 个服务器" % len([s for s in self.servers if s["enabled"]])

    def snapshot(self) -> dict:
        with self.lock:
            servers_snap = [dict(s) for s in self.servers]
            running = self.running
            progress = self.progress
        rows = []
        for i, srv in enumerate(servers_snap):
            st = compute_stats(srv["samples"])
            ok_samples = [{"rtt": s["rtt"], "offset": s["offset"]}
                          for s in srv["samples"] if s.get("ok")]
            rows.append({
                "idx": i,
                "name": srv["host"] or "(空)",
                "enabled": srv["enabled"],
                "status": srv["status"] if srv["enabled"] else "未启用",
                "color": COLORS[i % len(COLORS)],
                "stats": st,
                "samples": ok_samples,
            })
        # 排名（仅对成功者，按评分升序）
        ranked = [r for r in rows if r["stats"].get("score") is not None]
        ranked.sort(key=lambda r: r["stats"]["score"])
        for pos, r in enumerate(ranked):
            r["rank"] = pos + 1
        for r in rows:
            r.setdefault("rank", None)
        return {
            "running": running,
            "progress": progress,
            "defaults": DEFAULT_SERVERS,
            "servers": rows,
        }

    def export_csv(self) -> str:
        with self.lock:
            servers_snap = [dict(s) for s in self.servers]
        buf = io.StringIO()
        w = csv.writer(buf)
        w.writerow(["服务器", "状态", "成功样本", "总样本", "层级", "参考源",
                    "平均RTT(ms)", "最小RTT(ms)", "最大RTT(ms)", "RTT标准差(ms)",
                    "偏移均值(ms)", "抖动(ms)", "根离散(ms)", "评分"])
        for i, srv in enumerate(servers_snap):
            st = compute_stats(srv["samples"])
            w.writerow([
                srv["host"], srv["status"], st["ok"], st["total"], st["stratum"], st["ref"],
                _fmt(st["rtt_avg"]), _fmt(st["rtt_min"]), _fmt(st["rtt_max"]), _fmt(st["rtt_std"]),
                _fmt(st["offset_avg"]), _fmt(st["jitter"]), _fmt(st["root_disp"], 3), _fmt(st["score"]),
            ])
        return buf.getvalue()


def _fmt(v, nd=2):
    return ("%.{}f".format(nd)) % v if isinstance(v, (int, float)) else "-"


# ----------------------------------------------------------------------------
# 前端页面（HTML + Canvas 手绘图表，无第三方依赖）
# ----------------------------------------------------------------------------
PAGE = r"""<!doctype html>
<html lang="zh">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>NTP Benchmark · 服务器延时与时钟稳定性对比</title>
<style>
  :root {
    --bg: #f6f8fb;
    --bg-grad: radial-gradient(1200px 600px at 0% -10%, #e0e9ff 0%, transparent 50%),
               radial-gradient(1000px 500px at 100% 0%, #ffe7f0 0%, transparent 45%);
    --card: rgba(255,255,255,0.85);
    --card-solid: #ffffff;
    --line: #e6e9f0;
    --line-strong: #d6dbe6;
    --txt: #0f172a;
    --txt-2: #475569;
    --muted: #8894a8;
    --accent: #4f46e5;
    --accent-2: #06b6d4;
    --accent-grad: linear-gradient(135deg, #6366f1 0%, #06b6d4 100%);
    --ok: #10b981;
    --warn: #f59e0b;
    --err: #ef4444;
    --shadow-sm: 0 1px 2px rgba(15,23,42,.04), 0 1px 3px rgba(15,23,42,.06);
    --shadow-md: 0 4px 6px -1px rgba(15,23,42,.06), 0 10px 24px -8px rgba(15,23,42,.10);
    --shadow-lg: 0 10px 30px -10px rgba(79,70,229,.25);
    --radius: 14px;
    --radius-sm: 9px;
  }
  @media (prefers-color-scheme: dark) {
    :root {
      --bg: #0b1020;
      --bg-grad: radial-gradient(1200px 600px at 0% -10%, #1e1b4b 0%, transparent 50%),
                 radial-gradient(1000px 500px at 100% 0%, #3b0764 0%, transparent 45%);
      --card: rgba(20,26,45,0.75);
      --card-solid: #141a2d;
      --line: #232b45;
      --line-strong: #2f3a5c;
      --txt: #e6eaf3;
      --txt-2: #b6c0d4;
      --muted: #7b87a1;
      --accent: #818cf8;
      --accent-2: #22d3ee;
      --shadow-sm: 0 1px 2px rgba(0,0,0,.3);
      --shadow-md: 0 4px 6px -1px rgba(0,0,0,.35), 0 10px 24px -8px rgba(0,0,0,.5);
      --shadow-lg: 0 10px 30px -10px rgba(0,0,0,.6);
    }
  }
  * { box-sizing: border-box; }
  html, body { height: 100%; }
  body {
    margin: 0;
    font-family: "Inter", "Segoe UI", "PingFang SC", "Microsoft YaHei", system-ui, -apple-system, sans-serif;
    background: var(--bg);
    background-image: var(--bg-grad);
    background-attachment: fixed;
    color: var(--txt);
    -webkit-font-smoothing: antialiased;
    font-feature-settings: "cv11", "ss01";
    font-size: 14px;
    line-height: 1.55;
  }

  /* ---------- Header ---------- */
  header {
    position: sticky; top: 0; z-index: 50;
    backdrop-filter: saturate(180%) blur(14px);
    -webkit-backdrop-filter: saturate(180%) blur(14px);
    background: color-mix(in oklab, var(--card-solid) 78%, transparent);
    border-bottom: 1px solid var(--line);
  }
  .head-inner {
    max-width: 1440px; margin: 0 auto;
    padding: 14px 24px;
    display: flex; align-items: center; gap: 14px;
  }
  .logo {
    width: 38px; height: 38px; border-radius: 11px;
    background: var(--accent-grad);
    display: grid; place-items: center;
    color: #fff; font-weight: 700; font-size: 18px;
    box-shadow: var(--shadow-lg);
    flex-shrink: 0;
  }
  .head-text h1 { margin: 0; font-size: 16px; font-weight: 650; letter-spacing: -0.01em; }
  .head-text p { margin: 2px 0 0; color: var(--muted); font-size: 12px; }
  .head-badge {
    margin-left: auto;
    display: inline-flex; align-items: center; gap: 6px;
    padding: 6px 12px; border-radius: 999px;
    background: color-mix(in oklab, var(--accent) 10%, transparent);
    color: var(--accent); font-size: 12px; font-weight: 600;
    border: 1px solid color-mix(in oklab, var(--accent) 22%, transparent);
  }
  .pulse {
    width: 7px; height: 7px; border-radius: 50%;
    background: currentColor;
    box-shadow: 0 0 0 0 currentColor;
    animation: pulse 1.8s infinite;
  }
  @keyframes pulse {
    0% { box-shadow: 0 0 0 0 color-mix(in oklab, currentColor 60%, transparent); }
    70% { box-shadow: 0 0 0 8px transparent; }
    100% { box-shadow: 0 0 0 0 transparent; }
  }

  /* ---------- Layout ---------- */
  .wrap { max-width: 1440px; margin: 0 auto; padding: 22px 24px 60px; }

  .card {
    background: var(--card);
    backdrop-filter: blur(10px);
    -webkit-backdrop-filter: blur(10px);
    border: 1px solid var(--line);
    border-radius: var(--radius);
    padding: 18px 20px;
    margin-bottom: 18px;
    box-shadow: var(--shadow-sm);
    transition: box-shadow .25s, transform .25s;
  }
  .card:hover { box-shadow: var(--shadow-md); }
  .card h2 {
    margin: 0 0 14px;
    font-size: 14px; font-weight: 650;
    letter-spacing: -0.005em;
    display: flex; align-items: center; gap: 8px;
    color: var(--txt);
  }
  .card h2::before {
    content: ""; width: 3px; height: 14px; border-radius: 2px;
    background: var(--accent-grad);
  }
  .card h2 .sub { font-weight: 400; color: var(--muted); font-size: 12px; margin-left: auto; }

  /* ---------- Controls ---------- */
  .controls {
    display: grid;
    grid-template-columns: repeat(4, minmax(0,1fr));
    gap: 10px;
  }
  @media (max-width: 900px) { .controls { grid-template-columns: repeat(2, minmax(0,1fr)); } }
  @media (max-width: 480px) { .controls { grid-template-columns: 1fr; } }

  .srow {
    display: flex; align-items: center; gap: 8px;
    padding: 8px 10px;
    border: 1px solid var(--line);
    border-radius: var(--radius-sm);
    background: color-mix(in oklab, var(--card-solid) 60%, transparent);
    transition: border-color .2s, box-shadow .2s, background .2s;
  }
  .srow:hover { border-color: var(--line-strong); }
  .srow:focus-within {
    border-color: color-mix(in oklab, var(--accent) 55%, transparent);
    box-shadow: 0 0 0 3px color-mix(in oklab, var(--accent) 15%, transparent);
  }
  .srow.disabled { opacity: .5; }
  .srow input[type=text] {
    flex: 1; min-width: 0;
    padding: 5px 8px;
    border: 0; outline: 0;
    background: transparent;
    color: var(--txt);
    font-size: 12.5px;
    font-family: inherit;
  }
  .srow input[type=text]::placeholder { color: var(--muted); }

  /* 自定义 checkbox */
  .cb {
    appearance: none; -webkit-appearance: none;
    width: 16px; height: 16px; border-radius: 5px;
    border: 1.5px solid var(--line-strong);
    background: var(--card-solid);
    cursor: pointer; position: relative;
    flex-shrink: 0;
    transition: all .2s;
  }
  .cb:checked {
    background: var(--accent-grad);
    border-color: transparent;
  }
  .cb:checked::after {
    content: "";
    position: absolute; left: 4.5px; top: 1.5px;
    width: 5px; height: 9px;
    border: solid #fff;
    border-width: 0 2px 2px 0;
    transform: rotate(45deg);
  }

  /* ---------- Settings ---------- */
  .settings {
    display: flex; flex-wrap: wrap; gap: 20px; align-items: center;
    margin-top: 16px; padding-top: 16px;
    border-top: 1px dashed var(--line);
  }
  .field { display: flex; align-items: center; gap: 8px; }
  .field label { font-size: 12px; color: var(--txt-2); font-weight: 500; }
  .field input {
    width: 72px; padding: 6px 10px;
    border: 1px solid var(--line); border-radius: 8px;
    background: var(--card-solid); color: var(--txt);
    font-size: 12.5px; font-family: inherit;
    transition: border-color .2s, box-shadow .2s;
  }
  .field input:focus {
    outline: 0; border-color: var(--accent);
    box-shadow: 0 0 0 3px color-mix(in oklab, var(--accent) 15%, transparent);
  }

  /* ---------- Buttons ---------- */
  .btns { display: flex; gap: 10px; margin-top: 16px; flex-wrap: wrap; align-items: center; }
  button {
    display: inline-flex; align-items: center; gap: 7px;
    padding: 9px 18px;
    border: 0; border-radius: 10px;
    background: var(--accent-grad);
    color: #fff; font-size: 13px; font-weight: 600;
    font-family: inherit; cursor: pointer;
    box-shadow: var(--shadow-lg);
    transition: transform .15s, box-shadow .2s, filter .2s;
    white-space: nowrap;
  }
  button:hover:not(:disabled) { transform: translateY(-1px); filter: brightness(1.05); }
  button:active:not(:disabled) { transform: translateY(0); }
  button:disabled { opacity: .45; cursor: not-allowed; box-shadow: none; }
  button.sec {
    background: color-mix(in oklab, var(--card-solid) 90%, transparent);
    color: var(--txt-2);
    border: 1px solid var(--line-strong);
    box-shadow: var(--shadow-sm);
  }
  button.sec:hover:not(:disabled) { background: var(--card-solid); color: var(--txt); }
  button svg { width: 14px; height: 14px; }

  .progress {
    margin-left: auto;
    display: inline-flex; align-items: center; gap: 8px;
    color: var(--txt-2); font-size: 12.5px;
    padding: 7px 14px; border-radius: 999px;
    background: color-mix(in oklab, var(--card-solid) 70%, transparent);
    border: 1px solid var(--line);
  }
  .progress .dot {
    width: 8px; height: 8px; border-radius: 50%;
    background: var(--muted);
  }
  .progress.running .dot {
    background: var(--accent); animation: pulse 1.4s infinite;
  }
  .progress.done .dot { background: var(--ok); }

  /* ---------- Table ---------- */
  .table-wrap {
    overflow: auto; max-height: 380px;
    border-radius: var(--radius-sm);
    border: 1px solid var(--line);
  }
  table { width: 100%; border-collapse: separate; border-spacing: 0; font-size: 12.5px; }
  thead th {
    position: sticky; top: 0; z-index: 2;
    background: color-mix(in oklab, var(--card-solid) 96%, transparent);
    backdrop-filter: blur(8px);
    color: var(--muted); font-weight: 600;
    text-align: center; padding: 11px 12px;
    font-size: 11.5px; letter-spacing: .02em; text-transform: uppercase;
    border-bottom: 1px solid var(--line);
    white-space: nowrap;
  }
  tbody td {
    padding: 10px 12px; text-align: center;
    border-bottom: 1px solid var(--line);
    color: var(--txt-2); white-space: nowrap;
    font-variant-numeric: tabular-nums;
  }
  tbody tr { transition: background .15s; }
  tbody tr:hover { background: color-mix(in oklab, var(--accent) 5%, transparent); }
  tbody tr:last-child td { border-bottom: 0; }
  td.name {
    text-align: left; max-width: 200px; overflow: hidden;
    text-overflow: ellipsis; color: var(--txt); font-weight: 550;
  }
  .dot { display: inline-block; width: 9px; height: 9px; border-radius: 50%; margin-right: 7px; vertical-align: middle; box-shadow: 0 0 0 3px color-mix(in oklab, currentColor 0%, transparent); }
  .pill {
    display: inline-block; padding: 2px 8px; border-radius: 999px;
    font-size: 11px; font-weight: 600;
  }
  .pill.ok { background: color-mix(in oklab, var(--ok) 14%, transparent); color: var(--ok); }
  .pill.warn { background: color-mix(in oklab, var(--warn) 14%, transparent); color: var(--warn); }
  .pill.err { background: color-mix(in oklab, var(--err) 14%, transparent); color: var(--err); }
  .pill.mute { background: color-mix(in oklab, var(--muted) 14%, transparent); color: var(--muted); }
  .rank {
    display: inline-grid; place-items: center;
    width: 22px; height: 22px; border-radius: 7px;
    font-size: 11px; font-weight: 700;
    background: color-mix(in oklab, var(--accent) 12%, transparent);
    color: var(--accent);
  }
  .rank.gold { background: linear-gradient(135deg,#fbbf24,#f59e0b); color: #fff; box-shadow: 0 2px 8px rgba(245,158,11,.4); }
  .rank.silver { background: linear-gradient(135deg,#cbd5e1,#94a3b8); color: #fff; }
  .rank.bronze { background: linear-gradient(135deg,#fdba74,#ea580c); color: #fff; }

  /* ---------- Charts ---------- */
  .charts { display: grid; grid-template-columns: 1fr; gap: 18px; }
  @media (min-width: 1100px) { .charts { grid-template-columns: 1fr 1fr; } }
  .chart-card { padding: 18px 20px 14px; }
  .chart-title {
    display: flex; align-items: center; gap: 8px;
    margin-bottom: 6px;
  }
  .chart-title .t { font-size: 14px; font-weight: 650; }
  .chart-title .hint-inline { font-size: 11px; color: var(--muted); font-weight: 400; margin-left: auto; }
  .toggle {
    display: inline-flex; align-items: center; gap: 6px;
    font-size: 11.5px; color: var(--txt-2); font-weight: 500;
    cursor: pointer; user-select: none;
    padding: 4px 10px; border-radius: 999px;
    border: 1px solid var(--line);
    background: color-mix(in oklab, var(--card-solid) 70%, transparent);
    transition: all .2s;
  }
  .toggle:hover { border-color: var(--line-strong); }
  .toggle input { display: none; }
  .toggle .sw {
    width: 26px; height: 15px; border-radius: 999px;
    background: var(--line-strong); position: relative;
    transition: background .2s;
  }
  .toggle .sw::after {
    content: ""; position: absolute; top: 2px; left: 2px;
    width: 11px; height: 11px; border-radius: 50%;
    background: #fff; transition: transform .2s;
    box-shadow: 0 1px 2px rgba(0,0,0,.2);
  }
  .toggle input:checked + .sw { background: var(--accent); }
  .toggle input:checked + .sw::after { transform: translateX(11px); }

  canvas { width: 100%; height: 340px; display: block; }

  /* ---------- Hint ---------- */
  .hint {
    color: var(--muted); font-size: 12px; line-height: 1.7;
    padding: 14px 18px;
    background: color-mix(in oklab, var(--card-solid) 55%, transparent);
    border-left: 3px solid color-mix(in oklab, var(--accent) 50%, transparent);
    border-radius: 0 var(--radius-sm) var(--radius-sm) 0;
    margin: 4px 0 0;
  }
  .hint b { color: var(--txt-2); font-weight: 600; }

  /* ---------- Empty state ---------- */
  .empty {
    text-align: center; color: var(--muted);
    padding: 24px 0; font-size: 12.5px;
  }

  /* scrollbar */
  ::-webkit-scrollbar { width: 10px; height: 10px; }
  ::-webkit-scrollbar-track { background: transparent; }
  ::-webkit-scrollbar-thumb {
    background: color-mix(in oklab, var(--muted) 40%, transparent);
    border-radius: 999px; border: 3px solid transparent;
    background-clip: padding-box;
  }
  ::-webkit-scrollbar-thumb:hover { background: color-mix(in oklab, var(--muted) 60%, transparent); background-clip: padding-box; }
</style>
</head>
<body>
<header>
  <div class="head-inner">
    <div class="logo">N</div>
    <div class="head-text">
      <h1>NTP Benchmark</h1>
      <p>服务器延时 · 时钟源稳定性对比</p>
    </div>
    <div class="head-badge"><span class="pulse"></span><span id="badgeText">就绪</span></div>
  </div>
</header>

<div class="wrap">
  <div class="card">
    <h2>服务器配置<span class="sub">勾选启用 · 最多 8 个</span></h2>
    <div class="controls" id="controls"></div>
    <div class="settings">
      <div class="field"><label>采样次数</label><input id="samples" type="number" min="3" max="60" value="12"></div>
      <div class="field"><label>采样间隔 (s)</label><input id="interval" type="number" min="0.2" step="0.1" value="1.0"></div>
      <div class="field"><label>超时 (s)</label><input id="timeout" type="number" min="1" step="0.5" value="3.0"></div>
    </div>
    <div class="btns">
      <button id="startBtn">
        <svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2.5" stroke-linecap="round" stroke-linejoin="round"><polygon points="5 3 19 12 5 21 5 3"/></svg>
        开始 Benchmark
      </button>
      <button id="stopBtn" class="sec" disabled>
        <svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2.5" stroke-linecap="round" stroke-linejoin="round"><rect x="6" y="6" width="12" height="12" rx="1"/></svg>
        停止
      </button>
      <button id="resetBtn" class="sec">
        <svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2.5" stroke-linecap="round" stroke-linejoin="round"><path d="M3 12a9 9 0 1 0 3-6.7L3 8"/><path d="M3 3v5h5"/></svg>
        重置
      </button>
      <button id="exportBtn" class="sec">
        <svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2.5" stroke-linecap="round" stroke-linejoin="round"><path d="M21 15v4a2 2 0 0 1-2 2H5a2 2 0 0 1-2-2v-4"/><polyline points="7 10 12 15 17 10"/><line x1="12" y1="15" x2="12" y2="3"/></svg>
        导出 CSV
      </button>
      <span class="progress" id="progress"><span class="dot"></span><span id="progressText">就绪</span></span>
    </div>
  </div>

  <div class="card">
    <h2>对比表<span class="sub">综合评分 = 平均 RTT + 抖动，越小越好</span></h2>
    <div class="table-wrap">
      <table>
        <thead><tr>
          <th>服务器</th><th>状态</th><th>成功/总</th><th>层级</th><th>参考源</th>
          <th>平均 RTT</th><th>最小</th><th>最大</th>
          <th>偏移</th><th>抖动</th><th>根离散</th><th>评分</th><th>排名</th>
        </tr></thead>
        <tbody id="tbody"></tbody>
      </table>
    </div>
  </div>

  <div class="charts">
    <div class="card chart-card">
      <div class="chart-title">
        <span class="t">延时趋势</span>
        <span class="hint-inline">RTT · ms</span>
      </div>
      <canvas id="lat"></canvas>
    </div>
    <div class="card chart-card">
      <div class="chart-title">
        <span class="t">时钟稳定性</span>
        <label class="toggle" title="切换显示绝对偏移值">
          <input type="checkbox" id="absOffset">
          <span class="sw"></span>
          <span>绝对偏移</span>
        </label>
      </div>
      <canvas id="stab"></canvas>
    </div>
  </div>

  <p class="hint">
    <b>RTT</b> = 客户端到服务器的往返路径延时；<b>offset</b> = 客户端相对服务器时钟偏移；
    <b>jitter</b> = offset 标准差；<b>根离散</b> = 服务器自报的时钟离散度。
    稳定性图每根蜡烛的<b>上下影线</b>为 offset 的「最小~最大」区间、<b>箱体</b>为「均值 ± 抖动 (±1σ)」、<b>横线</b>为均值。
    默认视图已「以各自均值为中心（去均值）」绘制，只放大波动幅度，避免各服务器固定时钟偏差把真实波动压成一条扁线；
    勾选「绝对偏移」可查看绝对 offset。
  </p>
</div>

<script>
const COLORS = ["#6366f1","#f59e0b","#10b981","#ef4444","#8b5cf6","#06b6d4","#ec4899","#14b8a6"];
let DEFAULTS = __DEFAULTS__;
let state = { running:false, servers:[] };

const cssVar = n => getComputedStyle(document.documentElement).getPropertyValue(n).trim();
const fmt = (v, nd=2) => (typeof v === "number") ? v.toFixed(nd) : "-";
const escapeHtml = s => String(s).replace(/[&<>"]/g, c=>({'&':'&amp;','<':'&lt;','>':'&gt;','"':'&quot;'}[c]));

function buildControls(){
  const c = document.getElementById("controls");
  c.innerHTML = "";
  for(let i=0;i<8;i++){
    const row = document.createElement("div");
    row.className = "srow";
    const cb = document.createElement("input");
    cb.type = "checkbox"; cb.checked = true; cb.id = "en"+i; cb.className = "cb";
    const inp = document.createElement("input");
    inp.type = "text"; inp.value = DEFAULTS[i] || ""; inp.id = "host"+i;
    inp.placeholder = "NTP 服务器地址";
    cb.addEventListener("change", ()=>{
      row.classList.toggle("disabled", !cb.checked);
    });
    row.appendChild(cb); row.appendChild(inp);
    c.appendChild(row);
  }
}

function collectConfig(){
  const hosts=[], enabled=[];
  for(let i=0;i<8;i++){
    hosts.push(document.getElementById("host"+i).value);
    enabled.push(document.getElementById("en"+i).checked);
  }
  return {
    hosts, enabled,
    samples: parseInt(document.getElementById("samples").value)||12,
    interval: parseFloat(document.getElementById("interval").value)||1.0,
    timeout: parseFloat(document.getElementById("timeout").value)||3.0,
  };
}

async function api(path, method="GET", body=null){
  const opt = { method, headers:{"Content-Type":"application/json"} };
  if(body) opt.body = JSON.stringify(body);
  const r = await fetch(path, opt);
  if(path.endsWith(".csv")) return r;
  return r.json();
}

function statusPill(s){
  const map = {
    "完成":"ok", "运行中":"warn", "查询中":"warn",
    "错误":"err", "未启用":"mute", "等待":"mute",
    "已停止":"mute", "空":"mute"
  };
  const cls = map[s] || "mute";
  return '<span class="pill '+cls+'">'+escapeHtml(s)+'</span>';
}

function renderTable(servers){
  const tb = document.getElementById("tbody");
  tb.innerHTML = "";
  if(!servers.length){
    tb.innerHTML = '<tr><td colspan="13"><div class="empty">尚未开始，点击「开始 Benchmark」获取数据</div></td></tr>';
    return;
  }
  servers.forEach(s=>{
    const st = s.stats;
    const rankCls = s.rank===1?"gold":s.rank===2?"silver":s.rank===3?"bronze":"";
    const tr = document.createElement("tr");
    tr.innerHTML =
      '<td class="name"><span class="dot" style="background:'+s.color+'"></span>'+escapeHtml(s.name)+'</td>'+
      '<td>'+statusPill(st.status)+'</td>'+
      '<td>'+st.ok+'/'+st.total+'</td>'+
      '<td>'+st.stratum+'</td>'+
      '<td>'+escapeHtml(String(st.ref))+'</td>'+
      '<td>'+fmt(st.rtt_avg)+'</td>'+
      '<td>'+fmt(st.rtt_min)+'</td>'+
      '<td>'+fmt(st.rtt_max)+'</td>'+
      '<td>'+fmt(st.offset_avg,3)+'</td>'+
      '<td>'+fmt(st.jitter,3)+'</td>'+
      '<td>'+fmt(st.root_disp,3)+'</td>'+
      '<td>'+fmt(st.score)+'</td>'+
      '<td>'+(s.rank!=null?('<span class="rank '+rankCls+'">'+s.rank+'</span>'):"-")+'</td>';
    tb.appendChild(tr);
  });
}

function setupCanvas(cv){
  const dpr = window.devicePixelRatio || 1;
  const w = cv.clientWidth, h = cv.clientHeight;
  cv.width = w*dpr; cv.height = h*dpr;
  const ctx = cv.getContext("2d");
  ctx.setTransform(dpr,0,0,dpr,0,0);
  ctx.clearRect(0,0,w,h);
  return {ctx,w,h};
}

function roundRect(ctx, x, y, w, h, r){
  if (h < 2*r) r = h/2;
  ctx.beginPath();
  ctx.moveTo(x+r,y);
  ctx.arcTo(x+w,y,x+w,y+h,r);
  ctx.arcTo(x+w,y+h,x,y+h,r);
  ctx.arcTo(x,y+h,x,y,r);
  ctx.arcTo(x,y,x+w,y,r);
  ctx.closePath();
}

function hexA(hex,a){
  const r=parseInt(hex.slice(1,3),16), g=parseInt(hex.slice(3,5),16), b=parseInt(hex.slice(5,7),16);
  return "rgba("+r+","+g+","+b+","+a+")";
}

function drawLatency(servers){
  const cv = document.getElementById("lat");
  const {ctx,w,h} = setupCanvas(cv);
  const padL=58, padR=16, padT=26, padB=52;
  const x0=padL, x1=w-padR, y0=padT, y1=h-padB;

  const txtColor = cssVar("--txt") || "#0f172a";
  const mutedColor = cssVar("--muted") || "#8894a8";
  const lineColor = cssVar("--line") || "#e6e9f0";

  let maxR=1, maxN=1;
  servers.forEach(s=>{
    if(!s.samples || !s.samples.length) return;
    if(s.samples.length>maxN) maxN=s.samples.length;
    s.samples.forEach(p=>{ if(p.rtt>maxR) maxR=p.rtt; });
  });
  maxR = Math.max(maxR,1)*1.15;
  const nDiv = 4;

  // 网格 + Y 轴刻度
  ctx.font = "11px Inter, Segoe UI, sans-serif";
  ctx.textBaseline = "middle";
  for(let i=0;i<=nDiv;i++){
    const yy = y0 + (y1-y0)*i/nDiv;
    const val = maxR*(nDiv-i)/nDiv;
    ctx.strokeStyle = lineColor; ctx.lineWidth = 1;
    ctx.beginPath(); ctx.moveTo(x0, yy+.5); ctx.lineTo(x1, yy+.5); ctx.stroke();
    ctx.fillStyle = mutedColor; ctx.textAlign = "right";
    ctx.fillText(val.toFixed(1), x0-8, yy);
  }
  // X 轴刻度
  ctx.textAlign = "center"; ctx.textBaseline = "top";
  const step = maxN<=12 ? 1 : Math.ceil(maxN/12);
  for(let i=1;i<=maxN;i+=step){
    const xx = maxN>1 ? x0+(x1-x0)*((i-1)/(maxN-1)) : (x0+x1)/2;
    ctx.fillStyle = mutedColor;
    ctx.fillText(String(i), xx, y1+8);
  }
  ctx.fillStyle = mutedColor;
  ctx.textAlign = "center";
  ctx.fillText("采样序号", (x0+x1)/2, y1+30);

  // 数据点足够时绘制
  const hasData = servers.some(s=>s.samples && s.samples.length);
  if(!hasData){
    ctx.fillStyle = mutedColor;
    ctx.font = "13px Inter, Segoe UI, sans-serif";
    ctx.textAlign = "center"; ctx.textBaseline = "middle";
    ctx.fillText("暂无数据", w/2, h/2);
    return;
  }

  // 绘制每条线
  servers.forEach(s=>{
    if(!s.samples || !s.samples.length) return;
    const pts = s.samples;
    const coords = pts.map((p,i)=>{
      const xx = pts.length>1 ? x0+(x1-x0)*(i/(pts.length-1)) : (x0+x1)/2;
      const yy = y1-(y1-y0)*(p.rtt/maxR);
      return [xx,yy];
    });
    // 渐变填充
    if(coords.length > 1){
      const grad = ctx.createLinearGradient(0, y0, 0, y1);
      grad.addColorStop(0, hexA(s.color, .18));
      grad.addColorStop(1, hexA(s.color, 0));
      ctx.fillStyle = grad;
      ctx.beginPath();
      ctx.moveTo(coords[0][0], y1);
      coords.forEach(c=>ctx.lineTo(c[0], c[1]));
      ctx.lineTo(coords[coords.length-1][0], y1);
      ctx.closePath();
      ctx.fill();
    }
    // 线
    ctx.strokeStyle = s.color; ctx.lineWidth = 2;
    ctx.lineJoin = "round"; ctx.lineCap = "round";
    ctx.beginPath();
    coords.forEach((c,i)=> i?ctx.lineTo(c[0],c[1]):ctx.moveTo(c[0],c[1]));
    ctx.stroke();
    // 点
    coords.forEach(c=>{
      ctx.fillStyle = "#fff";
      ctx.beginPath(); ctx.arc(c[0], c[1], 3.2, 0, Math.PI*2); ctx.fill();
      ctx.fillStyle = s.color;
      ctx.beginPath(); ctx.arc(c[0], c[1], 2, 0, Math.PI*2); ctx.fill();
    });
  });

  // 图例
  ctx.font = "11px Inter, Segoe UI, sans-serif";
  ctx.textBaseline = "middle";
  const items = servers.filter(s=>s.samples && s.samples.length);
  let ly = 6, lx = x0;
  items.forEach(s=>{
    const label = s.name.length > 22 ? s.name.slice(0,21)+"…" : s.name;
    const tw = ctx.measureText(label).width;
    const itemW = 16 + tw + 16;
    if(lx + itemW > x1){ lx = x0; ly += 18; }
    // 色块
    ctx.fillStyle = s.color;
    roundRect(ctx, lx, ly+1, 10, 10, 3); ctx.fill();
    ctx.fillStyle = txtColor;
    ctx.textAlign = "left";
    ctx.fillText(label, lx+15, ly+7);
    lx += itemW;
  });
}

function drawStability(servers){
  const cv = document.getElementById("stab");
  const {ctx,w,h} = setupCanvas(cv);
  const padL=64, padR=16, padT=30, padB=96;
  const x0=padL, x1=w-padR, y0=padT, y1=h-padB;

  const txtColor = cssVar("--txt") || "#0f172a";
  const txt2 = cssVar("--txt-2") || "#475569";
  const mutedColor = cssVar("--muted") || "#8894a8";
  const lineColor = cssVar("--line") || "#e6e9f0";

  let absMode = false;
  try { absMode = document.getElementById("absOffset").checked; } catch(e){}

  const data = [];
  servers.forEach(s=>{
    if(!s.samples || s.samples.length===0) return;
    const offs = s.samples.map(p=>p.offset);
    const n = offs.length;
    const omin = Math.min(...offs), omax = Math.max(...offs);
    const omean = offs.reduce((a,b)=>a+b,0)/n;
    let v=0; for(const x of offs) v += (x-omean)*(x-omean);
    const ostd = Math.sqrt(v/n);
    if(absMode){
      data.push({s, lo:omin, hi:omax, mean:omean, std:ostd, absMean:omean});
    } else {
      const dev = offs.map(x=>x-omean);
      data.push({s, lo:Math.min(...dev), hi:Math.max(...dev), mean:0, std:ostd, absMean:omean});
    }
  });

  if(!data.length){
    ctx.fillStyle = mutedColor;
    ctx.font = "13px Inter, Segoe UI, sans-serif";
    ctx.textAlign = "center"; ctx.textBaseline = "middle";
    ctx.fillText("暂无数据", w/2, h/2);
    return;
  }

  // Y 范围
  let yMin, yMax;
  if(absMode){
    yMin=0; yMax=0;
    data.forEach(d=>{ yMin=Math.min(yMin,d.lo); yMax=Math.max(yMax,d.hi); });
    const p=(yMax-yMin)*0.12 || 0.1; yMin-=p; yMax+=p;
  } else {
    let m=0.001;
    data.forEach(d=>{ m=Math.max(m, Math.abs(d.lo), Math.abs(d.hi)); });
    const p=m*0.18; yMin=-(m+p); yMax=(m+p);
  }
  const Y = val => y1 - (y1-y0)*((val-yMin)/(yMax-yMin));

  // 网格
  ctx.font = "11px Inter, Segoe UI, sans-serif";
  ctx.textBaseline = "middle";
  for(let i=0;i<=5;i++){
    const yy = y0+(y1-y0)*i/5;
    const val = yMax-(yMax-yMin)*i/5;
    ctx.strokeStyle = lineColor; ctx.lineWidth = 1;
    ctx.beginPath(); ctx.moveTo(x0, yy+.5); ctx.lineTo(x1, yy+.5); ctx.stroke();
    ctx.fillStyle = mutedColor; ctx.textAlign = "right";
    ctx.fillText(val.toFixed(3), x0-8, yy);
  }
  // 零线
  const yz = Y(0);
  ctx.strokeStyle = lineColor; ctx.setLineDash([5,4]); ctx.lineWidth = 1.5;
  ctx.beginPath(); ctx.moveTo(x0, yz+.5); ctx.lineTo(x1, yz+.5); ctx.stroke();
  ctx.setLineDash([]);

  const n = data.length;
  const slot = (x1-x0)/n;
  const bw = Math.min(slot*0.5, 64);

  data.forEach((d,i)=>{
    const cx = x0 + slot*(i+0.5);
    const col = d.s.color;

    // 影线
    ctx.strokeStyle = col; ctx.lineWidth = 1.5;
    ctx.lineCap = "round";
    const yh = Y(d.hi), yl = Y(d.lo);
    ctx.beginPath(); ctx.moveTo(cx, yh); ctx.lineTo(cx, yl); ctx.stroke();
    ctx.beginPath(); ctx.moveTo(cx-6, yh); ctx.lineTo(cx+6, yh); ctx.stroke();
    ctx.beginPath(); ctx.moveTo(cx-6, yl); ctx.lineTo(cx+6, yl); ctx.stroke();

    // 箱体（带渐变）
    const ybLo = Y(Math.min(d.mean-d.std, d.hi));
    const ybHi = Y(Math.max(d.mean+d.std, d.lo));
    const boxH = ybLo - ybHi;
    const grad = ctx.createLinearGradient(0, ybHi, 0, ybLo);
    grad.addColorStop(0, hexA(col, .45));
    grad.addColorStop(1, hexA(col, .22));
    ctx.fillStyle = grad;
    roundRect(ctx, cx-bw/2, ybHi, bw, Math.max(boxH,1), 4);
    ctx.fill();
    ctx.strokeStyle = hexA(col, .8); ctx.lineWidth = 1;
    roundRect(ctx, cx-bw/2, ybHi, bw, Math.max(boxH,1), 4);
    ctx.stroke();

    // 均值横线
    const ym = Y(d.mean);
    ctx.strokeStyle = txtColor; ctx.lineWidth = 2; ctx.lineCap = "round";
    ctx.beginPath(); ctx.moveTo(cx-bw/2-8, ym); ctx.lineTo(cx+bw/2+8, ym); ctx.stroke();

    // 标注
    ctx.textAlign = "center"; ctx.textBaseline = "top";
    ctx.fillStyle = txtColor; ctx.font = "600 11px Inter, Segoe UI, sans-serif";
    const nameLabel = d.s.name.length > 16 ? d.s.name.slice(0,15)+"…" : d.s.name;
    ctx.fillText(nameLabel, cx, y1+10);
    ctx.fillStyle = txt2; ctx.font = "10px Inter, Segoe UI, sans-serif";
    ctx.fillText("μ="+d.absMean.toFixed(2), cx, y1+28);
    const rng = (d.lo>=0?" ":"")+d.lo.toFixed(3)+"  ~  "+(d.hi>=0?"+":"")+d.hi.toFixed(3);
    ctx.fillStyle = mutedColor;
    ctx.fillText(rng, cx, y1+44);
    ctx.fillText("抖动 ±"+d.std.toFixed(3), cx, y1+60);
  });
}

async function poll(){
  try {
    const snap = await api("/api/status");
    state = snap;
    renderTable(snap.servers);
    drawLatency(snap.servers);
    drawStability(snap.servers);

    const p = document.getElementById("progress");
    const pt = document.getElementById("progressText");
    const badge = document.getElementById("badgeText");
    pt.textContent = snap.progress;
    badge.textContent = snap.progress;
    p.classList.toggle("running", snap.running);
    p.classList.toggle("done", !snap.running && snap.progress.startsWith("完成"));

    document.getElementById("startBtn").disabled = snap.running;
    document.getElementById("stopBtn").disabled = !snap.running;
  } catch(e) { /* ignore */ }
  setTimeout(poll, 500);
}

document.getElementById("startBtn").onclick = async ()=>{ await api("/api/start","POST",collectConfig()); };
document.getElementById("stopBtn").onclick = async ()=>{ await api("/api/stop","POST"); };
document.getElementById("resetBtn").onclick = async ()=>{ await api("/api/reset","POST"); };
document.getElementById("exportBtn").onclick = ()=>{ window.location.href = "/api/export.csv"; };
document.getElementById("absOffset").addEventListener("change", ()=>{
  if(state.servers) drawStability(state.servers);
});

window.addEventListener("resize", ()=>{
  if(state.servers){
    drawLatency(state.servers);
    drawStability(state.servers);
  }
});

buildControls();
poll();
</script>
</body>
</html>
"""

# ----------------------------------------------------------------------------
# HTTP 服务
# ----------------------------------------------------------------------------
class Handler(BaseHTTPRequestHandler):
    bench = None  # 由主程序注入

    def _send(self, code, body, ctype="application/json; charset=utf-8"):
        if isinstance(body, str):
            body = body.encode("utf-8")
        self.send_response(code)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self):
        p = urlparse(self.path)
        if p.path in ("/", "/index.html"):
            html = PAGE.replace("__DEFAULTS__", json.dumps(DEFAULT_SERVERS, ensure_ascii=False))
            self._send(200, html, "text/html; charset=utf-8")
        elif p.path == "/api/status":
            self._send(200, json.dumps(self.bench.snapshot(), ensure_ascii=False))
        elif p.path == "/api/export.csv":
            csv_text = self.bench.export_csv()
            self.send_response(200)
            self.send_header("Content-Type", "text/csv; charset=utf-8")
            self.send_header("Content-Disposition", 'attachment; filename="ntp_benchmark.csv"')
            self.send_header("Content-Length", str(len(csv_text.encode("utf-8"))))
            self.end_headers()
            self.wfile.write(csv_text.encode("utf-8"))
        else:
            self._send(404, json.dumps({"error": "not found"}))

    def do_POST(self):
        p = urlparse(self.path)
        length = int(self.headers.get("Content-Length", 0) or 0)
        raw = self.rfile.read(length) if length else b"{}"
        try:
            data = json.loads(raw.decode("utf-8")) if raw else {}
        except Exception:
            data = {}
        if p.path == "/api/start":
            ok = self.bench.start(data)
            self._send(200, json.dumps({"started": ok}))
        elif p.path == "/api/stop":
            self.bench.stop()
            self._send(200, json.dumps({"ok": True}))
        elif p.path == "/api/reset":
            self.bench.reset()
            self._send(200, json.dumps({"ok": True}))
        else:
            self._send(404, json.dumps({"error": "not found"}))

    def log_message(self, *args):
        pass  # 静默日志


def main():
    import argparse
    ap = argparse.ArgumentParser(description="NTP 服务器 Benchmark (浏览器界面)")
    ap.add_argument("--host", default="127.0.0.1", help="监听地址 (默认 127.0.0.1)")
    ap.add_argument("--port", type=int, default=8000, help="监听端口 (默认 8000)")
    args = ap.parse_args()

    bench = Benchmark()
    Handler.bench = bench
    server = ThreadingHTTPServer((args.host, args.port), Handler)
    url = "http://%s:%d" % (args.host, args.port)
    print("NTP Benchmark 已启动：%s" % url)
    print("在浏览器中打开上面的地址即可使用。按 Ctrl+C 退出。")
    try:
        webbrowser.open_new(url)
    except Exception:
        pass
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\n已退出。")


# ----------------------------------------------------------------------------
# NTP 解析逻辑自检（无需网络）
# ----------------------------------------------------------------------------
def _self_test():
    def mk_resp(t1, t2):
        pkt = bytearray(48)
        pkt[0] = 0x1C
        pkt[1] = 2
        def enc(ts):
            sec = int(ts)
            frac = int((ts - sec) * 2 ** 32) & 0xFFFFFFFF
            return sec.to_bytes(4, "big") + frac.to_bytes(4, "big")
        pkt[32:40] = enc(t1)
        pkt[40:48] = enc(t2)
        return bytes(pkt)
    t0, t3 = 1000.50, 1000.70
    t1, t2 = 1000.55, 1000.65
    r = parse_ntp_response(mk_resp(t1, t2), t0, t3)
    assert abs(r["offset"] - 0.0) < 1e-6, r["offset"]
    assert abs(r["delay"] - 0.10) < 1e-6, r["delay"]
    print("[self-test] NTP 解析公式正确: offset=%.6f delay=%.6f" % (r["offset"], r["delay"]))


if __name__ == "__main__":
    _self_test()
    main()
