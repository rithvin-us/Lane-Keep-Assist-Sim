"""
01_extract_frames.py - turn a video into still frames for labelling.

  python 01_extract_frames.py --video videos/clip_A.mp4 --prefix A
  python 01_extract_frames.py --video videos/clip_B.mp4 --prefix B
  python 01_extract_frames.py --video videos/clip_A.mp4 --prefix A --zed-left     (ZED side-by-side recording)

Writes
  frames/all/A_000000.jpg ...     every candidate frame (default 5 per second)
  frames/round1/A_000000.jpg ...  every 6th candidate (default) = the frames you label first
  frames/A_info.json              the crop/resize settings, so later steps treat the video the same way

The number in the file name is the frame index in the source video.
"""
import argparse
import json
from pathlib import Path

import cv2

from lane_common import content_box_from_video, prepare_frame


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--video", required=True)
    ap.add_argument("--prefix", required=True, help="short tag for this video, e.g. A or B (letters/digits only)")
    ap.add_argument("--out", default="frames")
    ap.add_argument("--fps", type=float, default=5.0, help="candidate frames kept per second of video")
    ap.add_argument("--round1-every", type=int, default=6, help="every Nth candidate also goes to frames/round1")
    ap.add_argument("--zed-left", action="store_true", help="keep only the left half (ZED side-by-side video)")
    ap.add_argument("--no-autocrop", action="store_true", help="keep black letterbox bars")
    ap.add_argument("--max-width", type=int, default=1280, help="downscale wider frames to this width (0 = keep)")
    a = ap.parse_args()
    if not a.prefix.isalnum():
        ap.error("--prefix must be letters/digits only")

    cap = cv2.VideoCapture(a.video)
    if not cap.isOpened():
        raise SystemExit(f"cannot open {a.video}")
    src_fps = cap.get(cv2.CAP_PROP_FPS) or 30.0
    total = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    step = max(1, round(src_fps / a.fps))

    box = None
    if not a.no_autocrop:
        y0, y1, x0, x1 = content_box_from_video(cap)
        if a.zed_left:                              # the box was measured on the full frame; limit it to the left eye
            half = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH)) // 2
            x0, x1 = min(x0, half), min(x1, half)
        box = [y0, y1, x0, x1]

    out_all, out_r1 = Path(a.out) / "all", Path(a.out) / "round1"
    out_all.mkdir(parents=True, exist_ok=True)
    out_r1.mkdir(parents=True, exist_ok=True)

    i = kept = r1 = 0
    size = None
    while True:
        ok, frame = cap.read()
        if not ok:
            break
        if i % step == 0:
            # box is applied before the left-eye cut inside prepare_frame, so pass the pieces in the right order
            f = frame[:, : frame.shape[1] // 2] if a.zed_left else frame
            f = prepare_frame(f, box=box, zed_left=False, max_width=a.max_width)
            name = f"{a.prefix}_{i:06d}.jpg"
            cv2.imwrite(str(out_all / name), f, [cv2.IMWRITE_JPEG_QUALITY, 95])
            if kept % a.round1_every == 0:
                cv2.imwrite(str(out_r1 / name), f, [cv2.IMWRITE_JPEG_QUALITY, 95])
                r1 += 1
            kept += 1
            size = (f.shape[1], f.shape[0])
        i += 1
    cap.release()
    if kept == 0:
        raise SystemExit("no frames were read from the video")

    info = dict(video=str(a.video), prefix=a.prefix, source_fps=src_fps, source_frames=total, step=step,
                zed_left=a.zed_left, box=box, max_width=a.max_width, frame_size=size)
    (Path(a.out) / f"{a.prefix}_info.json").write_text(json.dumps(info, indent=2))
    print(f"{a.video}: {i} frames at {src_fps:.1f} fps -> kept every {step}th")
    print(f"  {kept} candidates in {out_all}   |   {r1} round-1 frames in {out_r1}   |   size {size[0]}x{size[1]}")
    if box:
        print(f"  cropped to rows {box[0]}-{box[1]}, columns {box[2]}-{box[3]}")


if __name__ == "__main__":
    main()
