#!/usr/bin/env python3
"""
HSV auto-tune — learns tighter color ranges from CONFIRMED ball pickups.

The Black Box saves FRONT-camera frames continuously and logs a
`ball_collected` event the moment a pickup is confirmed. This tool:

  1. finds every confirmed pickup in your sessions,
  2. opens the FRONT frame nearest each event,
  3. runs your CURRENT HSV range on it and finds the ball blob,
  4. accumulates the real HSV pixel values inside those blobs,
  5. proposes tightened S/V bounds (5th-95th percentile) that still contain
     the ball in every sampled frame, and prints paste-ready Kotlin.

The hue band is left alone (a ball's hue is what it is); saturation/value
bounds are where venue lighting noise lives. Review before pasting — this is
a PROPOSAL, not a command.

Usage:
    python3 hsv_autotune.py ../blackbox/teleop_1234 --hsv "14 100 100 35 255 255"
    python3 hsv_autotune.py ../blackbox --label Pollen --min-area 150
"""

import argparse
import json
import os

import bbdb
import sys

try:
    import cv2
    import numpy as np
except ImportError:
    sys.exit("This tool needs OpenCV + NumPy. Run:  pip install -r tools/requirements.txt")


def parse_hsv(text):
    vals = [float(x) for x in text.split()]
    if len(vals) != 6:
        sys.exit("--hsv needs exactly 6 numbers: H_MIN S_MIN V_MIN H_MAX S_MAX V_MAX")
    return (np.array(vals[:3], np.uint8), np.array(vals[3:], np.uint8))


def iter_sessions(paths):
    return bbdb.iter_sessions(paths)


def frames_index(records, camera="FRONT"):
    """(t, frame id) for every saved frame of one camera, sorted by time."""
    out = []
    for r in records:
        if r.get("frame") == camera and "frame_id" in r:
            out.append((r["t"], r["frame_id"]))
    return sorted(out)


def nearest_frame(idx, t):
    if not idx:
        return None
    best = min(idx, key=lambda f: abs(f[0] - t))
    return best if abs(best[0] - t) < 2000 else None  # within 2 s

def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("sessions", nargs="+",
                    help="Black Box session folder(s), or a parent folder of them")
    ap.add_argument("--hsv", default="14 100 100 35 255 255",
                    help="CURRENT ball range: H_MIN S_MIN V_MIN H_MAX S_MAX V_MAX (OpenCV scale)")
    ap.add_argument("--label", default=None,
                    help="only learn from ball_collected events of this label (default: all balls)")
    ap.add_argument("--min-area", type=float, default=150.0, help="min blob area to count a find")
    ap.add_argument("--pct-low", type=float, default=5.0, help="lower percentile for S/V bounds")
    ap.add_argument("--pct-high", type=float, default=95.0, help="upper percentile for S/V bounds")
    ap.add_argument("--out", default="hsv_autotune.json", help="where to save the proposal")
    args = ap.parse_args()

    lo, hi = parse_hsv(args.hsv)

    hsv_pixels = []  # all (H,S,V) rows inside detected ball blobs
    found, missed, no_frame = 0, 0, 0

    for session in iter_sessions(args.sessions):
        records = bbdb.load(session)
        idx = frames_index(records, "FRONT")
        if not idx:
            print("* {}: no FRONT frames — skipping".format(os.path.basename(session)))
            continue

        events = [r for r in records
                  if r.get("event") == "ball_collected" and
                  (args.label is None or r.get("label", "").find(args.label) >= 0)]
        for ev in events:
            fr = nearest_frame(idx, ev["t"])
            if fr is None:
                no_frame += 1
                continue
            jpeg = bbdb.frame_bytes(session, fr[1])
            img = cv2.imdecode(np.frombuffer(jpeg, np.uint8), cv2.IMREAD_COLOR) if jpeg else None
            if img is None:
                no_frame += 1
                continue
            hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
            mask = cv2.inRange(hsv, lo, hi)
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            contours = [c for c in contours if cv2.contourArea(c) >= args.min_area]
            if not contours:
                missed += 1
                continue
            c = max(contours, key=cv2.contourArea)
            x, y, w, h = cv2.boundingRect(c)
            roi = hsv[y:y + h, x:x + w]
            roi = roi[mask[y:y + h, x:x + w] > 0]
            if roi.size:
                hsv_pixels.append(roi.reshape(-1, 3))
                found += 1

    total = found + missed + no_frame
    print("==== HSV AUTO-TUNE ====")
    print("confirmed pickups sampled: {}  (found ball: {}, not found: {}, no frame: {})".format(
        total, found, missed, no_frame))
    if found < 5:
        sys.exit("! fewer than 5 usable samples — play more matches with the Black Box on "
                 "and make some confirmed pickups, then re-run.")

    pix = np.vstack(hsv_pixels).astype(np.float32)
    h_pct = np.percentile(pix[:, 0], [args.pct_low, args.pct_high])
    s_pct = np.percentile(pix[:, 1], [args.pct_low, args.pct_high])
    v_pct = np.percentile(pix[:, 2], [args.pct_low, args.pct_high])

    def fmt(x):
        return int(round(x))

    print("\nobserved ball pixels: {}   hue spread {:.0f}-{:.0f}".format(
        pix.shape[0], h_pct[0], h_pct[1]))
    print("proposed saturation: {}-{}   (was {}-{})".format(
        fmt(s_pct[0]), fmt(s_pct[1]), lo[1], hi[1]))
    print("proposed value:      {}-{}   (was {}-{})".format(
        fmt(v_pct[0]), fmt(v_pct[1]), lo[2], hi[2]))

    new_lo = (int(lo[0]), fmt(s_pct[0]), fmt(v_pct[0]))
    new_hi = (int(hi[0]), fmt(s_pct[1]), fmt(v_pct[1]))
    print("\n---- paste into globals.kt (review first!) ----")
    print("ColorRange(")
    print("    ColorSpace.HSV,")
    print("    Scalar({}, {}, {}),".format(float(new_lo[0]), float(new_lo[1]), float(new_lo[2])))
    print("    Scalar({}, {}, {})".format(float(new_hi[0]), float(new_hi[1]), float(new_hi[2])))
    print(")")
    print("------------------------------------------------")

    with open(args.out, "w") as f:
        json.dump({
            "current": {"lo": list(map(int, lo)), "hi": list(map(int, hi))},
            "proposed": {"lo": list(new_lo), "hi": list(new_hi)},
            "samples": int(pix.shape[0]),
            "found": found, "not_found": missed, "no_frame": no_frame,
        }, f, indent=2)
    print("* proposal saved to {}".format(args.out))
    print("* sanity-check with hsv_calibrator.py on a field video before pasting.")


if __name__ == "__main__":
    main()
