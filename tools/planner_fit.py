#!/usr/bin/env python3
"""
Planner constant fit — replaces my hand-guessed planner numbers with measured ones.

The StrategicPlanner's quality is limited by its cost estimates
(ROBOT_SPEED_CM_PER_SEC, PLANNER_COLLECT_HANDLE_SEC, ...). Those were guesses.
This tool measures the REAL ones from Black Box sessions:

  * robot speed      — from consecutive pose records (robust median)
  * collect cycle    — COLLECT_BALL decision -> ball_collected, minus travel
  * score cycle      — GO_SCORE decision -> fire_start -> last fire_ball_done,
                       minus travel  (= PLANNER_SCORE_TOTAL_SEC)
  * loader stroke    — loader_stroke -> shot_confirmed (stroke + dip timing)

and prints suggested globals.kt constants. Run it after every practice day;
paste the output; the planner's endgame arithmetic gets sharper each time.

Usage:
    python3 planner_fit.py ../blackbox/teleop_1234 [session2 ...]
    python3 planner_fit.py ../blackbox
"""

import argparse
import json
import math
import os
import sys

import bbdb


def iter_sessions(paths):
    return bbdb.iter_sessions(paths)


def load(path):
    return bbdb.load(path)


def median(xs):
    xs = sorted(xs)
    n = len(xs)
    return 0.0 if n == 0 else (xs[n // 2] if n % 2 else (xs[n // 2 - 1] + xs[n // 2]) / 2.0)


def path_cm_between(rows, t0, t1):
    """Distance traveled (cm) between two wallclock ms using pose rows."""
    pts = [(r["t"], r["poseXcm"], r["poseYcm"]) for r in rows
           if "poseXcm" in r and t0 <= r["t"] <= t1]
    total = 0.0
    for i in range(1, len(pts)):
        total += math.hypot(pts[i][1] - pts[i - 1][1], pts[i][2] - pts[i - 1][2])
    return total

def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("sessions", nargs="+",
                    help="Black Box session folder(s), or a parent folder of them")
    args = ap.parse_args()

    speeds = []          # cm/s segments
    collect_cycles = []  # seconds, decision -> ball_collected
    collect_travel = []  # cm traveled during those cycles
    score_cycles = []    # seconds, GO_SCORE decision -> last fire_ball_done
    score_travel = []    # cm traveled during those cycles
    strokes = []         # seconds, loader_stroke -> shot_confirmed

    for session in iter_sessions(args.sessions):
        records = load(session)
        rows = [r for r in records if "poseXcm" in r and "poseYcm" in r]

        # 1. speed from pose deltas (state rows are ~10 Hz)
        for i in range(1, len(rows)):
            dt = (rows[i]["t"] - rows[i - 1]["t"]) / 1000.0
            if dt <= 0 or dt > 1.0:
                continue
            d = math.hypot(rows[i]["poseXcm"] - rows[i - 1]["poseXcm"],
                           rows[i]["poseYcm"] - rows[i - 1]["poseYcm"])
            if d > 3.0:  # ignore noise & standstill
                speeds.append(d / dt)

        # 2/3. decision->outcome cycles (needs the shadow/selfplay planner events)
        events = [r for r in records if "event" in r]
        for i, ev in enumerate(events):
            if ev.get("event") != "planner_decision":
                continue
            action = ev.get("action")
            t0 = ev["t"]
            if action == "COLLECT_BALL":
                for later in events[i + 1:]:
                    if later.get("event") == "ball_collected":
                        collect_cycles.append((later["t"] - t0) / 1000.0)
                        collect_travel.append(path_cm_between(rows, t0, later["t"]))
                        break
            elif action == "GO_SCORE":
                fire_start = None
                last_done = None
                for later in events[i + 1:]:
                    e = later.get("event")
                    if e == "fire_start" and fire_start is None:
                        fire_start = later["t"]
                    elif e in ("fire_ball_done", "fire_done") and fire_start is not None:
                        last_done = later["t"]
                    elif e == "planner_decision" or (later["t"] - t0) > 60_000:
                        break
                if last_done is not None:
                    score_cycles.append((last_done - t0) / 1000.0)
                    score_travel.append(path_cm_between(rows, t0, last_done))

        # 4. loader stroke -> shot confirm
        for i, ev in enumerate(events):
            if ev.get("event") != "loader_stroke":
                continue
            for later in events[i + 1:]:
                if later.get("event") in ("shot_confirmed", "shot_missed"):
                    if later["t"] - ev["t"] < 10_000:
                        strokes.append((later["t"] - ev["t"]) / 1000.0)
                    break

    print("==== PLANNER CONSTANT FIT ====")
    if speeds:
        v = median(speeds)
        print("robot speed (median of {} segments):  {:.0f} cm/s".format(len(speeds), v))
    else:
        print("robot speed: no pose data found")
        v = None

    if collect_cycles and v:
        cyc = median(collect_cycles)
        travel = median(collect_travel)
        handle = max(0.5, cyc - travel / v)
        print("collect cycles (n={}):                 median {:.1f}s, travel {:.0f}cm "
              "-> handle ~{:.1f}s".format(len(collect_cycles), cyc, travel, handle))
    else:
        handle = None
        print("collect cycles: not enough data (need shadow/selfplay teleop sessions)")

    if score_cycles and v:
        cyc = median(score_cycles)
        travel = median(score_travel)
        score_total = max(2.0, cyc - travel / v)
        print("score cycles (n={}):                   median {:.1f}s, travel {:.0f}cm "
              "-> PLANNER_SCORE_TOTAL_SEC ~{:.1f}s".format(
                  len(score_cycles), cyc, travel, score_total))
    else:
        score_total = None
        print("score cycles: not enough data (need planner-driven scoring runs)")

    if strokes:
        print("loader stroke->confirm (n={}):         median {:.2f}s  "
              "(tune LOADER_STROKE_MS / SHOT_DETECT_WINDOW_MS if way off)".format(
                  len(strokes), median(strokes)))
    else:
        print("loader stroke->confirm: no strokes logged")

    print("\n---- suggested globals.kt updates ----")
    if v:
        print("const val ROBOT_SPEED_CM_PER_SEC = {:.0f}  // measured from {} segments".format(
            round(v), len(speeds)))
    if handle:
        print("const val PLANNER_COLLECT_HANDLE_SEC = {:.1f}".format(handle))
    if score_total:
        print("const val PLANNER_SCORE_TOTAL_SEC = {:.1f}".format(score_total))
    print("---------------------------------------")
    if not (v and handle and score_total):
        print("\n! some constants still missing data — run SelfPlay or shadow-mode "
              "teleop sessions that collect AND score, then re-run.")


if __name__ == "__main__":
    main()
