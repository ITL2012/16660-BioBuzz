# BioBuzz autonomy stack — legality review

**Scope:** the `feature/superhuman-autonomy` branch: Black Box full-match
recorder (state + both camera streams), WorldModel, StrategicPlanner,
shadow-mode planner, supervised-autonomy ("SelfPlay") teleop, and reflexes.

**Honest caveat up front:** the official BioBuzz Game Manual Part 1 could not
be fetched with my tooling during development (firstinspires.org served JS/404
walls; search engines blocked automated access). **Everything below is based
on the rules that have been stable across recent FTC seasons (DECODE, INTO THE
DEEP, CENTERSTAGE). You MUST verify each item against your copy of the BioBuzz
GM1 (FIRST Dashboard → FTC → Competition Materials) before using any of this
in an official match, and ideally ask the official Game Q&A.**

## Bottom line

Nothing in this branch adds radio hardware, external control, or anything that
leaves the robot during a match. The risky-but-probably-legal items are
(1) onboard video recording during a match and (2) fully autonomous software
during the driver-controlled period — both have been legal in past seasons
under the conditions noted below, and both are built here in the most
conservative form available.

## Feature-by-feature

| Feature | What it does | Rule area (VERIFY in BioBuzz GM1) | Assessment |
|---|---|---|---|
| Black Box state log | Writes ONE SQLite file (state, tracks, decisions, events, camera frames) to the Robot Controller's own storage | Robot Rules: permitted devices / ROBOT Controller | Very safe. Logging to the RC phone's storage is not a "device"; nothing transmits |
| Black Box frame capture | Saves subsampled JPEGs from the two USB webcams already attached to the RC | Robot Rules: cameras must connect to the Robot Controller; past manuals allow onboard cameras and video recording, and prohibit using video to communicate with the drive team mid-match | Safe as built: cameras are RC-attached (already true for the vision stack), recording stays onboard, nothing is viewed until after the match. Check the camera/recording rule in BioBuzz GM1 says the same |
| Shadow-mode planner | Pure logging of what the planner WOULD do during human teleop | none — no robot behavior changes | Safe. It's telemetry |
| WorldModel + StrategicPlanner | Onboard software decisions | Game Rules: driver-controlled period | Onboard autonomy during teleop has never been prohibited — rules require the ROBOT to be controlled through the Driver Station and to stop when commanded, not that a human move every stick. The deadman (below) keeps a human in the loop at all times |
| SelfPlay teleop (deadman) | Robot self-drives; human HOLDS left trigger, release = stop; Y = hint | Game Rules + Tournament Rules: driver control, disabling | Conservative design: a human hand is on the controller the entire match, and the robot still responds to the Driver Station stop/disable as always. VERIFY there is no BioBuzz-specific rule requiring continuous active driver input |
| Flywheel distance-based fire | Software-only | none | Safe |
| Tilt guard / jam reflex | Software-only, cuts power | Safety rules | Safe (safety-positive) |
| Neural nets (TFLite ball detector + Kotlin MLP shot predictor) | Inference only, entirely inside the RC app; no training, no network, no new hardware — the TFLite jar is just a software dependency | Robot Rules: permitted electronics/computing (all on the legal Robot Controller); nothing transmits | Safe as built. Authority ladder defaults to ADVISORY (predictions logged, behavior unchanged) — flip to REFEREE/PRIMARY only after the logged data proves the model |
| Post-match processing (`blackbox_builder.py`) | Runs on a laptop in the pit | Tournament Rules: between-match activity | Safe — analyzing your own logs between matches is ordinary pit work. Don't hold up the field or queue with it |
| ~~Remote operation~~ | — | explicitly NOT built | You're right, it's illegal: the robot must be controlled via the field Driver Station with no other radio links. Not attempted |

## What makes this defensible to an inspector

1. **Deadman:** in SelfPlay, a human hand is physically on the controller the
   entire match; releasing it stops the robot within one control loop (~20 ms).
2. **Driver Station supremacy:** every OpMode obeys normal stop/disable; the
   robot is 100% controlled through the legal control system.
3. **No new hardware:** same two RC-attached webcams, same hubs, same phone.
4. **No transmission:** the Black Box only writes local files; nothing
   streams, broadcasts, or leaves the robot while a match is in progress.
5. **Kill switches:** `BLACKBOX_ENABLED`, `SHADOW_PLANNER_ENABLED` in
   `globals.kt` turn each layer off with one line if an inspector or event
   manager is uncomfortable.

## Pre-match verification checklist (do this once, in writing)

- [ ] Locate the **camera / video recording** rule in BioBuzz GM1. Confirm:
      cameras attached to the Robot Controller are legal and onboard
      recording is permitted. Note any wording about not using recordings
      during the match — we comply (nothing is viewed mid-match).
- [ ] Locate the **"ROBOT must be controlled via the Driver Station"** rule
      (usually Robot or Game rules). Confirm nothing requires *continuous
      human input* (as opposed to the robot *responding to* the DS).
- [ ] Check the **autonomous period** rules don't extend restrictions into
      the driver-controlled period (we comply either way — autonomous-period
      autonomy is unambiguous).
- [ ] Check **radio/wireless** rules: confirm no rule prohibits writing files
      to the RC phone mid-match (nothing in past seasons did).
- [ ] Ask the **official Game Q&A** (ftc-qa.firstinspires.org), roughly:
      *"Is it legal for ROBOT software to act autonomously during the
      Driver-Controlled Period while a driver holds a deadman switch, and to
      record camera video to the Robot Controller's storage during a MATCH?"*
      Get the answer in writing and keep a printout with the robot.
- [ ] Tell your **event's head referee** before your first match with SelfPlay
      active. Compliance is judged by humans — surprise is how legal things
      become match problems.

## If any check fails

- Recording rule fails → set `BLACKBOX_ENABLED = false`, or keep state logs
  and drop only frame capture at official events.
- Teleop autonomy fails → run `BioBuzzTeleOp` (human-driven) with the shadow
  planner; the autonomy stack still lives in `BioBuzzAutonomous`, where it is
  unambiguously legal.
