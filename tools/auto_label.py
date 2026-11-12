#!/usr/bin/env python3
"""
Auto-labeler — turns Black Box frames into a YOLO training dataset.

The current HSV pipeline acts as the TEACHER: every saved FRONT frame gets the
same blob detection run on it, and each detected ball/flower becomes a YOLO
label. This does ~95% of the labeling for free; run label_review.py next to
fix the teacher's mistakes (lighting hallucinations, missed balls).

Class ids (must match NN_LABELS in globals.kt):
    0 Pollen, 1 Nectar Blue, 2 Nectar Red (both hue bands), 3 Flower

Frames with NO detections are kept (1 in --background-every) as empty labels —
backgrounds teach the detector what is NOT a ball.

Usage:
    python3 auto_label.py ../blackbox --out dataset
    python3 auto_label.py ../blackbox --out dataset --min-area 120 --epochs 80
"""

import argparse
import os

import bbdb
import random
import shutil
import sys

try:
    import cv2
    import numpy as np
except ImportError:
    sys.exit("This tool needs OpenCV + NumPy. Run:  pip install -r tools/requirements.txt")

CLASSES = ["Pollen", "Nectar Blue", "Nectar Red", "Flower"]

# Same HSV ranges as globals.kt (OpenCV hue 0-180)
RANGES = {
    0: [((14, 100, 100), (35, 255, 255))],                 # Pollen (yellow)
    1: [((100, 100, 100), (140, 255, 255))],               # Nectar Blue
    2: [((0, 100, 100), (12, 255, 255)),
        ((168, 100, 100), (180, 255, 255))],               # Nectar Red (wrapping)
    3: [((45, 50, 50), (80, 255, 255))],                   # Flower (green)
}


def detect(img, min_area):
    """Returns {class_id: [(x, y, w, h), ...]} using the HSV teacher."""
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    out = {}
    for cls, bands in RANGES.items():
        mask = np.zeros(hsv.shape[:2], np.uint8)
        for lo, hi in bands:
            mask |= cv2.inRange(hsv, np.array(lo, np.uint8), np.array(hi, np.uint8))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        boxes = []
        for c in contours:
            if cv2.contourArea(c) < min_area:
                continue
            x, y, w, h = cv2.boundingRect(c)
            boxes.append((x, y, w, h))
        if boxes:
            out[cls] = boxes
    return out

def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("sessions", nargs="+", help="Black Box session folder(s) or parent")
    ap.add_argument("--out", default="dataset", help="output dataset folder")
    ap.add_argument("--min-area", type=float, default=150.0, help="teacher blob noise floor (px^2)")
    ap.add_argument("--background-every", type=int, default=5,
                    help="keep 1-in-N empty frames as negative examples")
    ap.add_argument("--split", type=float, default=0.85, help="train fraction")
    ap.add_argument("--max-frames", type=int, default=4000, help="hard cap per run")
    args = ap.parse_args()

    os.makedirs(os.path.join(args.out, "images", "train"), exist_ok=True)
    os.makedirs(os.path.join(args.out, "images", "val"), exist_ok=True)
    os.makedirs(os.path.join(args.out, "labels", "train"), exist_ok=True)
    os.makedirs(os.path.join(args.out, "labels", "val"), exist_ok=True)

    rng = random.Random(42)
    counts = {c: 0 for c in CLASSES}
    frames_labeled = 0
    frames_background = 0
    empty_seen = 0
    total = 0

    for session in bbdb.iter_sessions(args.sessions):
            for frame_id, jpeg in bbdb.frames(session, "FRONT"):
                if total >= args.max_frames:
                    break
                total += 1
                img = cv2.imdecode(np.frombuffer(jpeg, np.uint8), cv2.IMREAD_COLOR)
                if img is None:
                    continue
                h_px, w_px = img.shape[:2]
                dets = detect(img, args.min_area)
                if not dets:
                    empty_seen += 1
                    if empty_seen % args.background_every != 0:
                        continue
                split = "train" if rng.random() < args.split else "val"
                stem = "{}_{:06d}".format(os.path.basename(session)[:24], frame_id)
                with open(os.path.join(args.out, "images", split, stem + ".jpg"), "wb") as f:
                    f.write(jpeg)
                lines = []
                for cls, boxes in dets.items():
                    for (x, y, w, h) in boxes:
                        cx, cy = x + w / 2.0, y + h / 2.0
                        lines.append("{:d} {:.6f} {:.6f} {:.6f} {:.6f}".format(
                            cls, cx / w_px, cy / h_px, w / w_px, h / h_px))
                        counts[CLASSES[cls]] += 1
                with open(os.path.join(args.out, "labels", split, stem + ".txt"), "w") as f:
                    f.write("\n".join(lines) + ("\n" if lines else ""))
                if dets:
                    frames_labeled += 1
                else:
                    frames_background += 1
            if total >= args.max_frames:
                break

    print("==== AUTO-LABEL DONE ====")
    print("frames processed: {}".format(total))
    print("  with objects:   {}  ({} boxes total)".format(
        frames_labeled, sum(counts.values())))
    print("  backgrounds:    {}".format(frames_background))
    for c, n in counts.items():
        print("  {:<12} {} boxes".format(c, n))
    if sum(counts.values()) == 0:
        sys.exit("! no labels produced — check that your Black Box sessions "
                 "contain FRONT frames and the HSV ranges match current lighting.")
    print("\nNEXT STEPS:")
    print("  1. python3 label_review.py {}   # fix the teacher's mistakes".format(args.out))
    print("  2. python3 train_detector.py {}  # train + export the TFLite model".format(args.out))


if __name__ == "__main__":
    main()
