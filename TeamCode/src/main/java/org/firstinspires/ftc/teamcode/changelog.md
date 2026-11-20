## Before you push something, please write a summary of your changes, if your changes are large enough to sanction one, about what things were implemented, new apis, etc.
### You can delete what the last person put here, that is in git history

---

## main: fix build, unify drive wiring, calibration-free defaults, full manual

### The build was broken
`main` did not compile. Commit 127bcbc copied teleop code from
`feature/superhuman-autonomy` without the three symbols that only exist on
that branch: `ControlFrame`, `HiveScoring.aprilTagEnabled`, and
`FlyWheel.ready()`.

- **Added `ControlFrame.kt`** — 20 ms motor-poll frame limiter (ported from
  the superhuman branch).
- **Added `HiveScoring.aprilTagEnabled`** — when false, `update()` returns
  null without scanning the AprilTag processor. The teleop enables it only
  while the shoot-align assist is held, instead of scanning every loop.
- **`FlyWheel.ready()` -> `FlyWheel.prime()`** in the teleop (main has
  `prime()`; `ready()` only existed on the superhuman branch).

### One source of truth for drive wiring
The teleop (`Drivetrain` in `motors.kt`) and the autonomous
(`roadrunner/MecanumDrive.kt`) each carried their own motor directions, and
they disagreed with each other, with the manual, and with the proven JJ
`TestTeleOP` scheme. They could not both be right.

- **Added `driveconfig.kt`** — the only place drive polarity and the mixing
  matrix live. Names, polarity flags, and `mix()` in one file.
- `Drivetrain` and `MecanumDrive` now both read it, so the teleop and the
  autonomous cannot disagree about which way the robot faces.
- Polarity defaults match JJ's proven scheme (FL/BL forward, FR/BR reverse).
- `MecanumDrive` also sets the matching Road Runner *encoder* direction.
  The SDK negates `currentPosition` for a reversed motor and RR's
  `RawEncoder` negates it again; matching the two keeps all four wheels
  counting UP when the robot drives forward, so the odometry stays correct.
- `driveTeleOp` now uses the shared matrix. The old inline copy had a
  different turn sign, so the "JJ raw-stick" toggle did not reproduce JJ's
  scheme despite the commit message claiming it did.
- The POSITION moves (`driveForward`/`driveBackward`/`driveRight`/`turn`) are
  affected by the same fix: they command all four wheels the same way, which
  only produces forward motion when the two sides' polarities match the
  physical mounting. With the old all-FORWARD flags those moves would not
  have driven straight. They now agree with the teleop and the autonomous.

### Calibration-free defaults
Three things needed a tuning session before. Now they do not.

- **Road Runner feedforward** (`kS`, `kV`) was `0.0` — every trajectory move
  commanded zero motor power, so the autonomous park move did nothing.
  `DrivetrainSpec` now derives `kV` from the motor's free speed and wheel
  diameter (`kV = inPerTick * (batteryV - kS) / freeSpeed`), with `kS` as the
  static-friction voltage. Derived from the hardware, not guessed.
- **Road Runner position gains** were `0.0` — the controller contributed
  nothing. Now conservative defaults (2.0 position, 0.5 velocity).
- **The shooter** always fired at full speed when the calibration curve was
  empty, which sails past a close hive. `ShooterModel` now falls back to a
  physics estimate from launch speed and range
  (`ShooterModel.estimatedRpmForDistance`), using
  `SHOOTER_FLYWHEEL_RADIUS_CM` and `SHOOTER_TRANSFER_EFFICIENCY`.

### Lessons carried over from pushbot-simple
Same physical drivetrain, so its lessons belong here. Ported as new files
rather than a merge (merging that branch would delete the neural stack, the
Black Box, the tools, and the docs).

- **Added `DriveDirectionTests.kt`**:
  - *Wheel Direction Test* — per-wheel truth test (A/B/X/Y = one wheel each,
    D-pad UP = all four). Works in raw power, so it still works with a dead
    encoder.
  - *Flywheel Direction Test* — spins each wheel alone; checks that they
    counter-rotate and that measured RPMs are positive.
  - *Drive Direction Check* — drives 50 cm and verifies the pose actually
    moved in x and not y.
- **Added `ReenterStartingAreaAuto.kt`** — "Simple 50cm Out-Back". The
  smallest autonomous that exercises the whole drive chain, on the same
  shared helper as the teleop.

### Flywheel spin direction
Was hardcoded in `FlyWheel` with no way to change it.

- **Added `FLYWHEEL_LEFT_REVERSED` / `FLYWHEEL_RIGHT_REVERSED`** in
  `globals.kt`. Flip one to flip a wheel; flip both to reverse the launch
  direction. Full procedure in README Chapter 10.4.

### Manual
`README.md` rewritten (1,436 lines). New **Chapter 0 "Start Here"** with a
10-minute no-calibration bring-up path, and directions on everything that
was previously missing or wrong: the drive wiring and how to flip a wheel,
how to swap the flywheel spin direction, the real hub port map, the two
different shooter range limits, and what still needs real measurement.
Stale content corrected: the old hardware table (117 RPM / 51.46:1 / 87 cm
chassis), the IMU orientation (actual mount is logo RIGHT / USB UP), the
shooter RPM steps, and the file layout.

### Tests
50 JVM tests, all passing (was 32). New `DriveConfigTest.kt` covers the
mixing matrix against JJ's reference scheme, the polarity flags, the
derived feedforward, and the shooter estimate.

### Competition-readiness work (the night before comp)

The robot had no code flashed and nothing tested, and the lead developer was
not on site. Three things were added so one person could get it working.

- **Added `PreFlightCheck.kt` — the "PRE-FLIGHT CHECK" OpMode.** Checks every
  subsystem and prints PASS/WARN/FAIL plus a READY / NOT READY verdict. FAIL
  means broken; WARN means missing but survivable. This is the first thing to
  run at the venue.
- **The IMU can no longer take the robot down.** Road Runner's
  `LazyHardwareMapImu.get()` calls `imu.initialize()`, which THROWS if the
  IMU is present but unhealthy. The localizers called it from their
  constructors, so a bad IMU could crash every OpMode — including the bench
  tests used to diagnose it. The IMU is now initialized eagerly inside a
  try/catch; a failure downgrades the robot to encoder-only heading and the
  pre-flight check reports a WARN with the reason.
- **The IMU yaw is zeroed at startup** (`MecanumDrive.resetHeading()`).
  Without a zero, heading is an arbitrary angle left over from whenever the
  hub was powered on, so field-centric driving and autonomous turns were off
  by an unknown amount. Zeroing makes heading repeatable. The IMU is also
  now an independent backup for heading when the dead wheels are absent
  (each covers what the other loses).
- **Added `scripts/`** — one-command setup and flashing for all three
  platforms: `setup-mac.sh`, `setup-linux.sh`, `setup-windows.ps1`,
  `deploy_to_hub.sh`, `deploy_to_hub.ps1`. The setup scripts generate
  `local.properties`, which is the #1 cause of "the code only works on my
  computer" (it is gitignored, so every new machine needs it). The deploy
  scripts verify the app actually started instead of trusting adb's
  "Success", and can wait for a flaky USB connection.
- **README.md rewritten as a 16-chapter operations manual** (~1,900 lines):
  a first-10-minutes path, a match-day runbook, setup from a fresh clone,
  flashing, the pre-flight check, hardware/wiring, drivetrain, the IMU and
  graceful degradation, shooter, vision, autonomous, teleop, a
  symptom-indexed troubleshooting chapter, extending the code, an appendix of
  every constant, and a glossary. Every constant name, value, OpMode name and
  file path in it was audited against the source.

### Known follow-ups (not done here)
- The hub's saved config (`DeathMetal.xml`) has no odometry pods, so Road
  Runner falls back to drive-motor odometry. `globals.kt` now names the pods
  `backleftodo` / `backrightodo` / `frontodo` — confirm those match the
  config actually used at the next meet.
- The pushbot branch's `ReenterStartingAreaAuto` used a timed move; this
  port uses an encoder position move instead, so the distance comes from the
  measured geometry rather than a time constant.
