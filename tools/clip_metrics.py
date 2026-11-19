"""clip_metrics.py - score run_lka_on_video results the same way for every detector.

    python3 tools/clip_metrics.py test_res/mobilenet backup_lane_detection/test_res/cnn ...

For every <folder>/**/lka_video_results.csv prints one row:
  both %    frames with both lines reported
  any %     frames with at least one line
  jumps     frame-to-frame steps of a reported line's offset > 0.3 m (visible flicker)
  swaps     steps > 1.0 m (the fit moved to another line)
  widthCV   std / median of the left-right pair width (stable lane = small)
  lka %     frames with the LKA ACTIVE or DEGRADED (state >= 2)
  det ms    median detector time; rest ms: bird's-eye + odometry + lane fit
Standard library only.
"""
import csv
import statistics
import sys
from pathlib import Path


def score(path):
    rows = list(csv.DictReader(open(path, newline="")))
    if not rows:
        return None
    f = lambda r, k: float(r[k]) if r.get(k) not in (None, "", "NaN") else float("nan")  # noqa: E731
    L = [(f(r, "LeftStrength") > 0, f(r, "LeftOffset_m")) for r in rows]
    R = [(f(r, "RightStrength") > 0, f(r, "RightOffset_m")) for r in rows]
    n = len(rows)
    both = sum(a[0] and b[0] for a, b in zip(L, R))
    anyl = sum(a[0] or b[0] for a, b in zip(L, R))
    jumps = swaps = 0
    for side in (L, R):
        for (ok0, y0), (ok1, y1) in zip(side, side[1:]):
            if ok0 and ok1:
                d = abs(y1 - y0)
                jumps += d > 0.3
                swaps += d > 1.0
    widths = [a[1] - b[1] for a, b in zip(L, R) if a[0] and b[0]]
    cv = statistics.pstdev(widths) / statistics.median(widths) if len(widths) > 2 else float("nan")
    det = statistics.median(f(r, "Detect_ms") for r in rows)
    rest = statistics.median(f(r, "BevOdometry_ms") + f(r, "LaneFit_ms") for r in rows)
    lka = sum(f(r, "LkaState") >= 2 for r in rows)
    return dict(frames=n, both=100 * both / n, any=100 * anyl / n, jumps=jumps, swaps=swaps, cv=cv,
                lka=100 * lka / n, det=det, rest=rest)


def main():
    print(f"{'run':62s} {'frames':>6s} {'both%':>6s} {'any%':>6s} {'jumps':>6s} {'swaps':>6s} "
          f"{'widthCV':>7s} {'lka%':>6s} {'det ms':>7s} {'rest ms':>7s}")
    for root in sys.argv[1:] or ["test_res"]:
        for p in sorted(Path(root).rglob("lka_video_results.csv")):
            s = score(p)
            if s:
                print(f"{str(p.parent)[-62:]:62s} {s['frames']:6d} {s['both']:6.1f} {s['any']:6.1f} {s['jumps']:6d} "
                      f"{s['swaps']:6d} {s['cv']:7.3f} {s['lka']:6.1f} {s['det']:7.1f} {s['rest']:7.1f}")


if __name__ == "__main__":
    main()
