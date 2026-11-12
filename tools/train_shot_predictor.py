#!/usr/bin/env python3
"""
Shot-outcome predictor trainer — neural net stage B.

Trains the tiny MLP the robot's ShotPredictor (neural.kt) runs in pure Kotlin:
P(shot scores) from (distance, RPM, battery voltage). Data comes from your
Black Box sessions: every `fire_start` -> `shot_confirmed/shot_missed` pair
with a live hive distance is one labeled sample. The trained weights export
as a small JSON you copy to the RC app's external storage (the folder that
contains blackbox/): <app files>/nn/shot_predictor.json

Usage:
    python3 train_shot_predictor.py ../blackbox
    python3 train_shot_predictor.py session1 session2 --hidden 8 --epochs 600
"""

import argparse
import json
import math
import os

import bbdb
import random
import sys

try:
    import numpy as np
except ImportError:
    sys.exit("This tool needs numpy:  pip install numpy")


def iter_sessions(paths):
    return bbdb.iter_sessions(paths)


def extract_samples(records):
    """One sample per shot: [distance_cm, rpm, battery_volts, hit(0/1)]."""
    samples = []
    ctx, volts = None, 0.0
    for r in records:
        if "batteryV" in r and isinstance(r["batteryV"], (int, float)) and r["batteryV"] > 0:
            volts = r["batteryV"]
        ev = r.get("event")
        if ev == "fire_start":
            ctx = (r.get("hiveCm"), r.get("rpm"))
        elif ev in ("shot_confirmed", "shot_missed") and ctx is not None:
            dist, rpm = ctx
            if dist is not None and rpm is not None and dist > 0 and rpm > 0:
                samples.append([float(dist), float(rpm), float(volts),
                                1.0 if ev == "shot_confirmed" else 0.0])
            ctx = None
    return samples

def sigmoid(z):
    return 1.0 / (1.0 + np.exp(-z))


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("sessions", nargs="+", help="Black Box session folder(s) or a parent folder")
    ap.add_argument("--hidden", type=int, default=8, help="hidden layer size (robot runs any)")
    ap.add_argument("--epochs", type=int, default=600)
    ap.add_argument("--lr", type=float, default=0.05)
    ap.add_argument("--min-samples", type=int, default=30,
                    help="shots needed before training (fewer = the model memorizes noise)")
    ap.add_argument("--out", default="shot_predictor.json")
    args = ap.parse_args()

    samples = []
    for session in iter_sessions(args.sessions):
        records = bbdb.load(session)
        got = extract_samples(records)
        print("* {}: {} usable shots".format(os.path.basename(session), len(got)))
        samples.extend(got)

    if len(samples) < args.min_samples:
        sys.exit("! only {} usable shots (<{}). Play more matches or run the Flywheel "
                 "Calibration OpMode with the Black Box on, then re-run.".format(
                     len(samples), args.min_samples))

    data = np.array(samples)
    X, y = data[:, :3], data[:, 3:4]
    hits = int(y.sum())
    print("\nsamples: {}  ({} hit / {} miss)".format(len(y), hits, len(y) - hits))
    if hits == 0 or hits == len(y):
        sys.exit("! all outcomes identical — nothing to learn from yet. Need both "
                 "made and missed shots.")

    # split + normalize (train stats only — the robot uses these exact values)
    rng = random.Random(42)
    idx = list(range(len(X)))
    rng.shuffle(idx)
    n_val = max(4, len(X) // 5)
    val_idx, train_idx = idx[:n_val], idx[n_val:]
    mean = X[train_idx].mean(axis=0)
    std = X[train_idx].std(axis=0) + 1e-6
    Xn = (X - mean) / std

    for name, s in zip(("distance", "rpm", "batteryV"), std):
        if s < 1e-3:
            print("! WARNING: feature '{}' is (nearly) constant in the training data — "
                  "the model can't use it, and predictions at other values will be "
                  "garbage. Collect more varied data.".format(name))

    H = args.hidden
    np.random.seed(42)
    w1 = np.random.randn(H, 3) * math.sqrt(2.0 / 3)
    b1 = np.zeros(H)
    w2 = np.random.randn(H) * math.sqrt(2.0 / H)
    b2 = 0.0

    # class weights so misses count even when rare
    w_pos = len(y) / (2.0 * max(1, hits))
    w_neg = len(y) / (2.0 * max(1, len(y) - hits))
    weights = np.where(y == 1, w_pos, w_neg)

    m = np.zeros_like(w1)
    mb1 = np.zeros_like(b1)
    m2 = np.zeros_like(w2)
    mb2 = 0.0
    mom = 0.9

    for epoch in range(args.epochs):
        order = list(train_idx)
        rng.shuffle(order)
        for i in order:
            x = Xn[i]
            h = np.tanh(w1 @ x + b1)
            p = sigmoid(w2 @ h + b2)
            g = (p - y[i, 0]) * weights[i, 0]
            gw2 = g * h
            gb2 = g
            gh = g * w2 * (1 - h * h)
            gw1 = np.outer(gh, x)
            gb1 = gh
            m = mom * m - args.lr * gw1
            mb1 = mom * mb1 - args.lr * gb1
            m2 = mom * m2 - args.lr * gw2
            mb2 = mom * mb2 - args.lr * gb2
            w1 += m
            b1 += mb1
            w2 += m2
            b2 += mb2

        if (epoch + 1) % 100 == 0:
            def acc(ixs):
                ps = sigmoid(np.tanh(Xn[ixs] @ w1.T + b1) @ w2 + b2)
                preds = (ps > 0.5).astype(float)
                return (preds == y[ixs, 0]).mean(), ps
            tr, _ = acc(train_idx)
            va, _ = acc(val_idx)
            print("epoch {:4d}  train acc {:.1%}  val acc {:.1%}".format(epoch + 1, tr, va))

    tr_ps = sigmoid(np.tanh(Xn[train_idx] @ w1.T + b1) @ w2 + b2)
    va_ps = sigmoid(np.tanh(Xn[val_idx] @ w1.T + b1) @ w2 + b2)
    print("\nfinal: train acc {:.1%}   val acc {:.1%}".format(
        ((tr_ps > 0.5).astype(float) == y[train_idx, 0]).mean(),
        ((va_ps > 0.5).astype(float) == y[val_idx, 0]).mean()))

    # sanity table: P(hit) over distances at a flat 4500 RPM, 12.0V
    print("\nsanity check (fixed 4500 RPM @ 12.0V):")
    for d in (80, 100, 120, 140):
        x = (np.array([float(d), 4500.0, 12.0]) - mean) / std
        p = sigmoid(w2 @ np.tanh(w1 @ x + b1) + b2)
        print("  {:>3}cm -> {:.0f}%".format(d, p * 100))

    out = {
        "mean": mean.tolist(), "std": std.tolist(),
        "w1": w1.tolist(), "b1": b1.tolist(),
        "w2": w2.tolist(), "b2": b2,
        "meta": {"samples": len(y), "hits": hits, "hidden": H},
    }
    with open(args.out, "w") as f:
        json.dump(out, f, indent=2)
    print("\n* model saved to {}".format(args.out))
    print("* copy it to the RC app's external storage as nn/shot_predictor.json "
          "(same folder level as blackbox/) — ShotPredictor loads it on next "
          "OpMode start. Keep SHOT_PREDICTOR_MODE = ADVISORY until the logged "
          "predictions look right, then flip to REFEREE.")


if __name__ == "__main__":
    main()
