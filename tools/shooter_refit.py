#!/usr/bin/env python3
"""
Shooter curve refit — learns the RPM-vs-distance curve from Black Box matches.

Every confirmed shot the robot logs is a free calibration sample:
    fire_start(rpm, hiveCm) -> shot_confirmed(dipRpm) | shot_missed

This tool scans one or more Black Box session folders, buckets CONFIRMED
shots by the live hive distance, averages the commanded RPM that made them,
and prints a refined SHOOTER_CALIBRATION block ready to paste into
globals.kt (ShooterModel interpolates arrays of ANY equal length, so the
refined curve can have more points than the original 6). It also reports
success rates per RPM band and battery-voltage correlation, so you can see
whether missed shots cluster at low voltage.

Usage:
    python3 shooter_refit.py ../blackbox/teleop_1234 [session2 ...]
    python3 shooter_refit.py ../blackbox --bin-cm 15 --min-samples 3
"""

import argparse
import json
import math
import os

import bbdb
import sys


def iter_sessions(paths):
    """Yields session database paths (or parents full of them)."""
    return bbdb.iter_sessions(paths)


def load(path):
    return bbdb.load(path)


def extract_shots(records):
    """Pairs each shot_confirmed/shot_missed with the nearest preceding
    fire_start context (rpm, distance) and the latest battery voltage."""
    shots = []
    ctx = {"rpm": None, "hiveCm": None}
    volts = 0.0
    for r in records:
        ev = r.get("event")
        if "batteryV" in r and isinstance(r["batteryV"], (int, float)) and r["batteryV"] > 0:
            volts = r["batteryV"]
        if ev == "fire_start":
            ctx = {"rpm": r.get("rpm"), "hiveCm": r.get("hiveCm")}
        elif ev in ("shot_confirmed", "shot_missed"):
            shots.append({
                "ok": ev == "shot_confirmed",
                "rpm": ctx.get("rpm"),
                "distanceCm": ctx.get("hiveCm"),
                "dipRpm": r.get("dipRpm"),
                "batteryV": volts,
                "t": r.get("t"),
            })
    return shots

def median(xs):
    xs = sorted(xs)
    n = len(xs)
    if n == 0:
        return 0.0
    return xs[n // 2] if n % 2 else (xs[n // 2 - 1] + xs[n // 2]) / 2.0


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("sessions", nargs="+",
                    help="Black Box session folder(s), or a parent folder of them")
    ap.add_argument("--bin-cm", type=float, default=10.0, help="distance bucket size, cm")
    ap.add_argument("--min-samples", type=int, default=2,
                    help="confirmed shots needed per bucket to trust it")
    ap.add_argument("--out", default=None, help="also save results to this JSON")
    args = ap.parse_args()

    shots = []
    for session in iter_sessions(args.sessions):
        n = len(shots)
        shots.extend(extract_shots(load(session)))
        print("* {}: {} shots".format(os.path.basename(session), len(shots) - n))

    if not shots:
        sys.exit("! no shot records found — play some matches (or run the "
                 "Flywheel Calibration OpMode) and re-run.")

    confirmed = [s for s in shots if s["ok"]]
    missed = [s for s in shots if not s["ok"]]
    print("\n==== SHOT LEDGER ====")
    print("confirmed: {}   missed: {}   ({:.0f}% success)".format(
        len(confirmed), len(missed),
        100.0 * len(confirmed) / max(1, len(shots))))

    # success rate + battery by RPM band
    bands = {}
    for s in shots:
        if s["rpm"] is None:
            continue
        key = int(round(s["rpm"] / 500.0) * 500)
        bands.setdefault(key, []).append(s)
    if bands:
        print("\nper-RPM-band success:")
        for k in sorted(bands):
            ss = bands[k]
            ok = sum(1 for s in ss if s["ok"])
            v = [s["batteryV"] for s in ss if s["batteryV"]]
            print("  {:>5} RPM: {:>3}/{} ok ({:.0f}%){}".format(
                k, ok, len(ss), 100.0 * ok / len(ss),
                "   avg battery {:.2f}V".format(sum(v) / len(v)) if v else ""))
    if missed:
        vm = [s["batteryV"] for s in missed if s["batteryV"]]
        vc = [s["batteryV"] for s in confirmed if s["batteryV"]]
        if vm and vc:
            print("\nbattery on misses: {:.2f}V vs confirms: {:.2f}V {}".format(
                median(vm), median(vc),
                "(check voltage sag if misses are clearly lower)"))

    # ---- distance-bucketed curve from CONFIRMED shots ----
    samples = [s for s in confirmed
               if s["distanceCm"] is not None and s["rpm"] is not None]
    print("\nconfirmed shots with live hive distance: {}".format(len(samples)))
    if len(samples) < 4:
        print("! not enough placed shots yet (<4) — no curve output. "
              "Shoot at the hive from a few distances and re-run.")
        return

    buckets = {}
    for s in samples:
        b = int(math.floor(s["distanceCm"] / args.bin_cm) * args.bin_cm)
        buckets.setdefault(b, []).append(s["rpm"])

    curve = []
    for b in sorted(buckets):
        rpms = buckets[b]
        if len(rpms) < args.min_samples:
            print("  bucket {}-{}cm: only {} sample(s) — skipped "
                  "(--min-samples {})".format(b, b + args.bin_cm, len(rpms), args.min_samples))
            continue
        curve.append((b + args.bin_cm / 2.0, sum(rpms) / len(rpms), len(rpms)))
    if len(curve) < 2:
        print("! trusted buckets: {} — still not enough for a curve".format(len(curve)))
        return

    # sort by RPM descending, like SHOOTER_CALIBRATION_RPM
    curve.sort(key=lambda c: -c[1])

    print("\n==== REFINED SHOOTER CURVE ====")
    for d, rpm, n in curve:
        print("  {:>6.1f} cm -> {:>6.0f} RPM   ({} samples)".format(d, rpm, n))
    print("\n---- paste into globals.kt ----")
    print("val SHOOTER_CALIBRATION_RPM = doubleArrayOf({})".format(
        ", ".join("{:.1f}".format(r) for _, r, _ in curve)))
    print("var SHOOTER_CALIBRATION_DISTANCE_CM = doubleArrayOf(")
    for d, rpm, n in curve:
        print("    {:.1f}, // {:.0f} RPM ({} shots)".format(d, rpm, n))
    print(")")
    print("--------------------------------\n")
    print("* review the numbers before pasting — the robot will fire from "
          "anywhere inside this new range.")

    if args.out:
        with open(args.out, "w") as f:
            json.dump({"curve": [{"distanceCm": d, "rpm": r, "samples": n}
                                 for d, r, n in curve]}, f, indent=2)
        print("* results saved to {}".format(args.out))


if __name__ == "__main__":
    main()
