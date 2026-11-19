"""
lane_node.py - live lane detection node: camera -> MobileNet U-Net (onnxruntime, CPU) -> MATLAB.

    # ZED 2i as a plain USB camera (no ZED SDK needed), left eye, HD720 side-by-side
    python realtime/lane_node.py --camera 0 --zed

    # same, with the factory calibration file of your camera (lens undistortion + exact focal length)
    python realtime/lane_node.py --camera 0 --zed --calib SN12345.conf

    # rehearsal without a camera: a recorded clip, played at its own speed, looped
    python realtime/lane_node.py --video ~/Downloads/college_lane.mp4

    # record the raw left-eye video while running (for run_lka_on_video afterwards)
    python realtime/lane_node.py --camera 0 --zed --record runs/live

The node serves one TCP client (MATLAB: scripts/run_lka_live.m) on 127.0.0.1:5555.
On connect it sends one JSON "hello" (image size and intrinsics at the network resolution),
then one message per processed frame:

    header  '<4sIdfffHHB' b'LKA1', frame id, capture time [s since node start],
                          preprocessing ms, network ms, capture->send age ms, H, W, has_rgb
    lane    H*W uint8     lane-line probability * 255, row-major, at the network size
    rgb     H*W*3 uint8   the camera frame at the network size (RGB), when has_rgb = 1

A grab thread keeps only the newest camera frame, so a slow step never builds up a queue:
the frame MATLAB receives is at most one camera period plus one processing step old.
Timing is printed every 2 s (capture -> sent age, preprocessing, network).
"""
import argparse
import configparser
import json
import socket
import struct
import sys
import threading
import time
from pathlib import Path

import cv2
import numpy as np
import onnxruntime as ort

HERE = Path(__file__).resolve().parent
DEPLOY = HERE.parent / "deploy"
sys.path.insert(0, str(HERE.parent / "training"))
from lane_common import content_box_from_video, overlay  # noqa: E402

HEADER = struct.Struct("<4sIdfffHHB")
T0 = time.monotonic()


# ----------------------------------------------------------------------------- camera sources
class LatestFrame:
    """Grab thread: always holds the newest frame (drops the rest)."""

    def __init__(self, cap, pace_fps=0.0, loop=False):
        self.cap, self.pace, self.loop = cap, pace_fps, loop
        self.frame, self.t, self.n, self.ok = None, 0.0, 0, True
        self.cond = threading.Condition()
        threading.Thread(target=self._run, daemon=True).start()

    def _run(self):
        next_t = time.monotonic()
        while self.ok:
            ok, f = self.cap.read()
            if not ok:
                if self.loop:
                    self.cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
                    continue
                self.ok = False
                break
            if self.pace:                       # a file plays at its own frame rate
                next_t += 1.0 / self.pace
                time.sleep(max(0.0, next_t - time.monotonic()))
            with self.cond:
                self.frame, self.t, self.n = f, time.monotonic() - T0, self.n + 1
                self.cond.notify_all()
        with self.cond:
            self.cond.notify_all()

    def wait_newer(self, n_seen, timeout=2.0):
        with self.cond:
            self.cond.wait_for(lambda: self.n > n_seen or not self.ok, timeout)
            return self.frame, self.t, self.n


def open_source(a):
    if a.video:
        cap = cv2.VideoCapture(str(Path(a.video).expanduser()))
        if not cap.isOpened():
            raise SystemExit(f"cannot open {a.video}")
        box = None if a.no_autocrop else content_box_from_video(cap)
        return LatestFrame(cap, pace_fps=cap.get(cv2.CAP_PROP_FPS) or 30.0, loop=True), box
    cap = cv2.VideoCapture(a.camera, cv2.CAP_V4L2) if sys.platform.startswith("linux") else cv2.VideoCapture(a.camera)
    if not cap.isOpened():
        raise SystemExit(f"cannot open camera {a.camera} (ls /dev/video*; is another program using it?)")
    if a.zed:                                   # ZED UVC modes: 2560x720 (HD720) and 3840x1080 (HD1080)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 2 * a.width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, a.height)
    else:
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, a.width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, a.height)
    cap.set(cv2.CAP_PROP_FPS, a.fps)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    w, h = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH)), int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    print(f"camera {a.camera}: {w}x{h} @ {cap.get(cv2.CAP_PROP_FPS):.0f} fps" + ("  (ZED side-by-side, left eye used)" if a.zed else ""))
    if a.zed and w != 2 * a.width:
        print(f"  warning: asked for {2 * a.width}x{a.height}; the ZED gives {w}x{h} (check the USB 3 port)")
    return LatestFrame(cap), None


def read_zed_calib(path, eye_w):
    """Left-camera intrinsics and distortion from a Stereolabs SN<serial>.conf file."""
    cp = configparser.ConfigParser()
    cp.read(path)
    sec = {2208: "LEFT_CAM_2K", 1920: "LEFT_CAM_FHD", 1280: "LEFT_CAM_HD", 672: "LEFT_CAM_VGA"}.get(eye_w)
    if sec is None or sec not in cp:
        raise SystemExit(f"{path}: no calibration section for a {eye_w} px wide eye")
    s = cp[sec]
    K = np.array([[float(s["fx"]), 0, float(s["cx"])], [0, float(s["fy"]), float(s["cy"])], [0, 0, 1]])
    D = np.array([float(s.get(k, 0)) for k in ("k1", "k2", "p1", "p2", "k3")])
    return K, D


# ----------------------------------------------------------------------------- network
class Net:
    def __init__(self, threads):
        cfg = json.loads((DEPLOY / "mobilenet.json").read_text())
        so = ort.SessionOptions()
        so.intra_op_num_threads = threads
        so.add_session_config_entry("session.intra_op.allow_spinning", "0")   # leave CPU to MATLAB
        self.sess = ort.InferenceSession(str(DEPLOY / cfg["onnx"]), so, providers=["CPUExecutionProvider"])
        self.h, self.w = cfg["input_size"]
        self.lane = cfg["lane_channel"] - 1
        self.road = cfg.get("road_channel", 2) - 1
        self.classes = cfg.get("classes", ["background", "road", "lane"])
        self.mean = np.array(cfg["mean"], np.float32)
        self.std = np.array(cfg["std"], np.float32)
        self.name = self.sess.get_inputs()[0].name

    def __call__(self, rgb_small):
        x = (rgb_small.astype(np.float32) / 255.0 - self.mean) / self.std
        y = self.sess.run(None, {self.name: x.transpose(2, 0, 1)[None]})[0][0]
        e = np.exp(y - y.max(0, keepdims=True))
        p = e / e.sum(0, keepdims=True)
        return p[self.lane], p.argmax(0).astype(np.uint8)


# ----------------------------------------------------------------------------- MATLAB link
class Link:
    def __init__(self, port):
        self.srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        try:
            self.srv.bind(("127.0.0.1", port))
        except OSError as e:
            raise SystemExit(f"port {port} is busy ({e.strerror}): another lane_node is running "
                             f"(pkill -f lane_node.py) or use --port")
        self.srv.listen(1)
        self.srv.setblocking(False)
        self.conn, self.hello, self.port = None, None, port

    def poll_accept(self):
        if self.conn is not None:
            return
        try:
            c, _ = self.srv.accept()
        except BlockingIOError:
            return
        c.setblocking(True)
        c.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        h = json.dumps(self.hello).encode()
        c.sendall(struct.pack("<I", len(h)) + h)
        self.conn = c
        print("MATLAB connected")

    def send(self, fid, t, pre_ms, net_ms, lane_u8, rgb_small):
        age_ms = 1000 * (time.monotonic() - T0 - t)
        if self.conn is None:
            return False
        h, w = lane_u8.shape
        msg = HEADER.pack(b"LKA1", fid, t, pre_ms, net_ms, age_ms, h, w, 1 if rgb_small is not None else 0)
        try:
            self.conn.sendall(msg + lane_u8.tobytes() + (rgb_small.tobytes() if rgb_small is not None else b""))
            return True
        except OSError:
            print("MATLAB disconnected, waiting again")
            self.conn.close()
            self.conn = None
            return False


# ----------------------------------------------------------------------------- main loop
def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    src = ap.add_mutually_exclusive_group(required=True)
    src.add_argument("--camera", type=int, help="V4L2 index (ls /dev/video*)")
    src.add_argument("--video", help="recorded clip instead of a camera (looped, real-time pace)")
    ap.add_argument("--zed", action="store_true", help="side-by-side stereo camera: use the left eye")
    ap.add_argument("--width", type=int, default=1280, help="width of ONE eye (ZED HD720: 1280)")
    ap.add_argument("--height", type=int, default=720)
    ap.add_argument("--fps", type=int, default=30)
    ap.add_argument("--calib", help="Stereolabs SN<serial>.conf: undistort + exact intrinsics")
    ap.add_argument("--hfov", type=float, default=0.0,
                    help="horizontal field of view [deg] when there is no --calib (ZED 2i HD720 ~ 102 unrectified)")
    ap.add_argument("--rate", type=float, default=20.0, help="max frames per second sent (LKA loop rate)")
    ap.add_argument("--threads", type=int, default=6, help="onnxruntime threads (leave some cores to MATLAB)")
    ap.add_argument("--port", type=int, default=5555)
    ap.add_argument("--no-rgb", action="store_true", help="send only the lane map (no odometry in MATLAB)")
    ap.add_argument("--no-autocrop", action="store_true")
    ap.add_argument("--show", action="store_true", help="preview window with the overlay (q quits)")
    ap.add_argument("--record", help="folder: raw left-eye video + frame times for offline replay")
    a = ap.parse_args()

    link = Link(a.port)                               # bind first: a busy port fails before the camera opens
    net = Net(a.threads)
    grab, box = open_source(a)
    first, _, _ = grab.wait_newer(0, timeout=5.0)
    if first is None:
        raise SystemExit("no frame from the source")
    eye = first[:, : first.shape[1] // 2] if a.zed else first
    if box is not None:
        eye = eye[box[0]:box[1], box[2]:box[3]]
    eh, ew = eye.shape[:2]

    maps, K = None, None
    if a.calib:
        K, D = read_zed_calib(a.calib, ew)
        maps = cv2.initUndistortRectifyMap(K, D, None, K, (ew, eh), cv2.CV_16SC2)
    elif a.hfov > 0:
        f = (ew / 2) / np.tan(np.radians(a.hfov) / 2)
        K = np.array([[f, 0, ew / 2], [0, f, eh / 2], [0, 0, 1]])
    sx, sy = net.w / ew, net.h / eh
    hello = {"width": net.w, "height": net.h, "source": a.video or f"camera {a.camera}", "eye": [ew, eh],
             "undistorted": bool(maps), "fps_limit": a.rate}
    if K is not None:                                # intrinsics at the network resolution
        hello.update(fx=K[0, 0] * sx, fy=K[1, 1] * sy, cx=K[0, 2] * sx, cy=K[1, 2] * sy)
    print("camera model sent to MATLAB:", json.dumps(hello))
    link.hello = hello
    print(f"waiting for MATLAB on 127.0.0.1:{a.port}  (scripts/run_lka_live.m)")

    rec, rec_log = None, None
    if a.record:
        out = Path(a.record)
        out.mkdir(parents=True, exist_ok=True)
        stamp = time.strftime("%Y%m%d_%H%M%S")
        rec = cv2.VideoWriter(str(out / f"live_{stamp}.mp4"), cv2.VideoWriter_fourcc(*"mp4v"), a.fps, (ew, eh))
        rec_log = open(out / f"live_{stamp}_frames.csv", "w")
        rec_log.write("frame,t_capture_s,pre_ms,net_ms,sent\n")
        print(f"recording to {out / f'live_{stamp}.mp4'}")

    n_seen, fid, period = 0, 0, 1.0 / a.rate if a.rate > 0 else 0.0
    stats, t_report, t_next = [], time.monotonic(), time.monotonic()
    try:
        while grab.ok:
            link.poll_accept()
            if period:
                time.sleep(max(0.0, t_next - time.monotonic()))
                t_next = max(t_next + period, time.monotonic() - period)
            frame, t_cap, n_seen = grab.wait_newer(n_seen)
            if frame is None:
                continue
            t1 = time.perf_counter()
            f = frame[:, : frame.shape[1] // 2] if a.zed else frame
            if box is not None:
                f = f[box[0]:box[1], box[2]:box[3]]
            if maps is not None:
                f = cv2.remap(f, maps[0], maps[1], cv2.INTER_LINEAR)
            if rec is not None:
                rec.write(f)
            small = cv2.cvtColor(cv2.resize(f, (net.w, net.h), interpolation=cv2.INTER_AREA), cv2.COLOR_BGR2RGB)
            t2 = time.perf_counter()
            lane, classes = net(small)
            t3 = time.perf_counter()
            lane_u8 = np.clip(lane * 255 + 0.5, 0, 255).astype(np.uint8)
            pre_ms, net_ms = 1000 * (t2 - t1), 1000 * (t3 - t2)
            sent = link.send(fid, t_cap, pre_ms, net_ms, lane_u8, None if a.no_rgb else small)
            age_ms = 1000 * (time.monotonic() - T0 - t_cap)
            stats.append((age_ms, pre_ms, net_ms))
            if rec_log:
                rec_log.write(f"{fid},{t_cap:.4f},{pre_ms:.2f},{net_ms:.2f},{int(sent)}\n")
            if a.show:
                view = overlay(cv2.cvtColor(small, cv2.COLOR_RGB2BGR), classes, net.classes)
                cv2.putText(view, f"net {net_ms:.0f} ms  age {age_ms:.0f} ms", (8, 20),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 2)
                cv2.imshow("lane_node", cv2.resize(view, None, fx=2, fy=2))
                if cv2.waitKey(1) & 0xFF == ord("q"):
                    break
            fid += 1
            if time.monotonic() - t_report >= 2.0 and stats:
                s = np.array(stats)
                print(f"{len(s) / (time.monotonic() - t_report):5.1f} fps | capture->sent p50 {np.median(s[:, 0]):5.1f} "
                      f"p95 {np.percentile(s[:, 0], 95):5.1f} ms | pre {np.median(s[:, 1]):4.1f} ms | "
                      f"net {np.median(s[:, 2]):5.1f} ms | MATLAB {'connected' if link.conn else 'not connected'}",
                      flush=True)
                stats, t_report = [], time.monotonic()
    except KeyboardInterrupt:
        pass
    finally:
        grab.ok = False
        if rec is not None:
            rec.release()
            rec_log.close()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
