"""LKA monitoring dashboard: a small local web server (Python standard library only).

    python tools/lka_dashboard.py [--port 8050]

then open http://127.0.0.1:8050 (MATLAB: start_lka_dashboard). It shows

* the live run published by MATLAB (lka_monitor_publish -> results/monitor/live.json and
  live.jpg): live camera (run_lka_live + the lane node), recorded video (run_lka_on_video)
  or the Unreal camera bench; phase, progress, annotated camera frame, steering / lane /
  speed / latency signals;
* the computer: CPU load every second (/proc/stat) and, when nvidia-smi exists, the GPU;
* finished results: every test_res/**/lka_video_results.csv (recorded track videos, with the
  annotated videos and graphs), every results/live/*/live_frames.csv (live camera sessions)
  and results/vision/lka_vision_kpis.csv (closed-loop Unreal camera runs).

The server binds to 127.0.0.1 only, answers only requests addressed to 127.0.0.1 / localhost
(Host header check against DNS rebinding) and serves files only from test_res/, results/ and
docs/ of this repository.
"""
import argparse
import csv
import json
import mimetypes
import os
import re
import shutil
import subprocess
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import unquote, urlparse

ROOT = Path(__file__).resolve().parents[1]
STATIC = Path(__file__).resolve().parent / "dashboard"
MONITOR = ROOT / "results" / "monitor"
SERVED = [ROOT / "test_res", ROOT / "results", ROOT / "docs"]

GPU_FIELDS = ["name", "utilization.gpu", "memory.used", "memory.total", "temperature.gpu",
              "power.draw", "clocks.sm", "clocks.max.sm", "pstate"]
_gpu = {"history": [], "latest": None, "error": None}
_gpu_lock = threading.Lock()
_last_good = {}          # live.json / live.jpg bytes of the last successful read
_RANGE = re.compile(r"^bytes=(\d*)-(\d*)$")


def read_live(path):
    # MATLAB replaces the file by renaming a temporary one; on Windows a read can hit that
    # moment (PermissionError / missing file): serve the previous copy then.
    try:
        data = path.read_bytes()
        _last_good[path.name] = data
        return data
    except OSError:
        return _last_good.get(path.name)


def _num(v):
    try:
        return float(v)
    except (TypeError, ValueError):
        return None


def _cpu_times():
    try:
        with open("/proc/stat") as f:
            v = [int(x) for x in f.readline().split()[1:]]
        return sum(v), v[3] + (v[4] if len(v) > 4 else 0)      # total, idle + iowait
    except (OSError, ValueError, IndexError):
        return None


def gpu_poller():
    """CPU load and nvidia-smi once per second; keeps the last 300 samples for the charts."""
    exe = shutil.which("nvidia-smi")
    prev = _cpu_times()
    while True:
        sample, err = None, None
        cur = _cpu_times()
        cpu = None
        if prev and cur and cur[0] > prev[0]:
            cpu = 100.0 * (1 - (cur[1] - prev[1]) / (cur[0] - prev[0]))
        prev = cur
        if exe is None:
            err = "no NVIDIA GPU (nvidia-smi not found): CPU only"
            sample = {"t": time.time(), "cpu": cpu, "cores": os.cpu_count()}
        else:
            try:
                out = subprocess.run(
                    [exe, "--query-gpu=" + ",".join(GPU_FIELDS), "--format=csv,noheader,nounits"],
                    capture_output=True, text=True, timeout=5,
                    creationflags=getattr(subprocess, "CREATE_NO_WINDOW", 0)).stdout.strip()
                vals = [v.strip() for v in out.splitlines()[0].split(",")]
                sample = dict(zip(GPU_FIELDS, vals))
                sample = {"t": time.time(), "name": sample["name"], "pstate": sample["pstate"],
                          "util": _num(sample["utilization.gpu"]), "memUsed": _num(sample["memory.used"]),
                          "memTotal": _num(sample["memory.total"]), "temp": _num(sample["temperature.gpu"]),
                          "power": _num(sample["power.draw"]), "clock": _num(sample["clocks.sm"]),
                          "clockMax": _num(sample["clocks.max.sm"]), "cpu": cpu, "cores": os.cpu_count()}
            except Exception as exc:  # keep polling, report the error
                err = str(exc)
                sample = {"t": time.time(), "cpu": cpu, "cores": os.cpu_count()}
        with _gpu_lock:
            _gpu["error"] = err
            if sample:
                _gpu["latest"] = sample
                _gpu["history"].append(sample)
                del _gpu["history"][:-300]
        time.sleep(1.0)


def read_csv(path):
    if not path.is_file():
        return []
    with open(path, newline="", encoding="utf-8-sig") as f:
        return list(csv.DictReader(f))


def _pct(rows, test):
    return 100.0 * sum(1 for r in rows if test(r)) / len(rows) if rows else None


def _median(vals):
    v = sorted(x for x in vals if x is not None)
    return v[len(v) // 2] if v else None


def _video_summary(csv_path):
    """One results row from a run_lka_on_video table (test_res/<...>/lka_video_results.csv)."""
    rows = read_csv(csv_path)
    if not rows:
        return None
    f = lambda r, k: _num(r.get(k))  # noqa: E731
    t = [f(r, "Time_s") or 0.0 for r in rows]
    detect = _median([f(r, "Detect_ms") for r in rows])
    rest = _median([(f(r, "BevOdometry_ms") or 0) + (f(r, "LaneFit_ms") or 0) for r in rows])
    speeds = [f(r, "Speed_kmh") for r in rows if f(r, "Speed_kmh") is not None]
    steer = [abs(f(r, "SteerCmd_deg") or 0) for r in rows]
    return {"Frames": len(rows), "Duration_s": t[-1] - t[0],
            "BothLines_pct": _pct(rows, lambda r: (f(r, "LeftStrength") or 0) > 0 and (f(r, "RightStrength") or 0) > 0),
            "AnyLine_pct": _pct(rows, lambda r: (f(r, "LeftStrength") or 0) > 0 or (f(r, "RightStrength") or 0) > 0),
            "LkaActive_pct": _pct(rows, lambda r: (f(r, "LkaState") or 0) >= 2),
            "Detect_ms": detect, "Matlab_ms": rest,
            "MaxSpeed_kmh": max(speeds) if speeds else None, "MaxSteerCmd_deg": max(steer) if steer else None}


def results_payload():
    videos = []
    for csv_path in sorted((ROOT / "test_res").rglob("lka_video_results.csv")):
        folder = csv_path.parent
        row = _video_summary(csv_path)
        if row is None:
            continue
        rel = folder.relative_to(ROOT / "test_res").as_posix()
        row["Video"] = rel
        files = {}
        for key, fname in [("video", "lka_video_annotated.mp4"), ("overview", "lka_video_overview.png"),
                           ("path", "lka_video_path.png"), ("csv", "lka_video_results.csv")]:
            if (folder / fname).is_file():
                files[key] = "/files/test_res/" + rel + "/" + fname
        videos.append({"row": row, "files": files})
    lives = []
    for csv_path in sorted((ROOT / "results" / "live").glob("*/live_frames.csv"), reverse=True):
        rows = read_csv(csv_path)
        if not rows:
            continue
        f = lambda r, k: _num(r.get(k))  # noqa: E731
        rel = csv_path.parent.relative_to(ROOT).as_posix()
        row = {"Session": csv_path.parent.name, "Frames": len(rows),
               "Duration_s": f(rows[-1], "Time_s"),
               "BothLines_pct": _pct(rows, lambda r: (f(r, "LeftOffset_m") or 0) != 0 and (f(r, "RightOffset_m") or 0) != 0),
               "Node_ms": _median([f(r, "Node_ms") for r in rows]),
               "Matlab_ms": _median([f(r, "Matlab_ms") for r in rows])}
        files = {}
        for key, fname in [("overview", "live_overview.png"), ("csv", "live_frames.csv"), ("replay", "live_results.csv")]:
            if (csv_path.parent / fname).is_file():
                files[key] = "/files/" + rel + "/" + fname
        lives.append({"row": row, "files": files})
    sims = []
    vis = ROOT / "results" / "vision"
    for row in read_csv(vis / "lka_vision_kpis.csv"):
        scen = row.get("Scenario", "")
        det = row.get("Detector", "")
        tag = (scen + "_" + row.get("Controller", "")).replace(" ", "").replace(",", "")
        files = {}
        for key, suffix in [("overview", "_overview.png"), ("perception", "_perception.png")]:
            p = vis / (tag + suffix)
            if p.is_file():
                files[key] = "/files/results/vision/" + p.name
        mp4 = vis / (scen + "_" + det + ".mp4")
        if mp4.is_file():
            files["video"] = "/files/results/vision/" + mp4.name
        sims.append({"row": row, "files": files})
    return {"videos": videos, "lives": lives, "sims": sims}


class Handler(BaseHTTPRequestHandler):
    server_version = "LKADashboard/1.0"

    def log_message(self, fmt, *args):  # quiet console
        pass

    def _send(self, code, body, ctype, extra=None):
        self.send_response(code)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        for k, v in (extra or {}).items():
            self.send_header(k, v)
        self.end_headers()
        if self.command != "HEAD":
            self.wfile.write(body)

    def _json(self, obj):
        self._send(200, json.dumps(obj).encode("utf-8"), "application/json")

    def _file(self, path):
        size = path.stat().st_size
        ctype = mimetypes.guess_type(path.name)[0] or "application/octet-stream"
        rng = self.headers.get("Range")
        if rng:
            m = _RANGE.match(rng.strip())             # one range only; else 416
            if not m or (m.group(1) == "" and m.group(2) == ""):
                self._send(416, b"", ctype, {"Content-Range": "bytes */%d" % size})
                return
            if m.group(1) == "":                       # suffix range: last N bytes
                start, end = max(0, size - int(m.group(2))), size - 1
            else:
                start = int(m.group(1))
                end = min(int(m.group(2)) if m.group(2) else size - 1, size - 1)
            if start > end or start >= size:
                self._send(416, b"", ctype, {"Content-Range": "bytes */%d" % size})
                return
            with open(path, "rb") as f:
                f.seek(start)
                body = f.read(end - start + 1)
            self._send(206, body, ctype, {"Content-Range": "bytes %d-%d/%d" % (start, end, size),
                                          "Accept-Ranges": "bytes"})
            return
        self._send(200, path.read_bytes(), ctype, {"Accept-Ranges": "bytes"})

    def do_HEAD(self):
        self.do_GET()

    def do_GET(self):
        host = (self.headers.get("Host") or "").lower()
        port = self.server.server_address[1]
        if host not in ("127.0.0.1:%d" % port, "localhost:%d" % port):
            self._send(403, b"forbidden host", "text/plain")
            return
        url = urlparse(self.path)
        route = unquote(url.path)
        try:
            if route in ("/", "/index.html"):
                self._file(STATIC / "index.html")
            elif route == "/api/live":
                raw = read_live(MONITOR / "live.json")
                if raw:
                    data = json.loads(raw.decode("utf-8"))
                    data["Age"] = time.time() - float(data.get("Updated") or 0)
                    data["FrameAvailable"] = (MONITOR / "live.jpg").is_file() or "live.jpg" in _last_good
                else:
                    data = {"Phase": "no run published yet", "Age": None}
                self._json(data)
            elif route == "/api/frame":
                raw = read_live(MONITOR / "live.jpg")
                if raw:
                    self._send(200, raw, "image/jpeg")
                else:
                    self._send(404, b"no frame", "text/plain")
            elif route == "/api/gpu":
                with _gpu_lock:
                    self._json({"latest": _gpu["latest"], "history": _gpu["history"][-120:],
                                "error": _gpu["error"]})
            elif route == "/api/results":
                self._json(results_payload())
            elif route.startswith("/files/"):
                target = (ROOT / route[len("/files/"):]).resolve()
                if not any(target.is_relative_to(d.resolve()) for d in SERVED) or not target.is_file():
                    self._send(404, b"not found", "text/plain")
                else:
                    self._file(target)
            else:
                self._send(404, b"not found", "text/plain")
        except (ConnectionError, BrokenPipeError):
            pass
        except Exception as exc:
            self._send(500, str(exc).encode("utf-8"), "text/plain")


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--port", type=int, default=8050)
    args = ap.parse_args()
    mimetypes.add_type("video/mp4", ".mp4")
    threading.Thread(target=gpu_poller, daemon=True).start()
    srv = ThreadingHTTPServer(("127.0.0.1", args.port), Handler)
    print("LKA dashboard on http://127.0.0.1:%d (Ctrl+C to stop)" % args.port, flush=True)
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
