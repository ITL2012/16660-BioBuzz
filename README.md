# FTC Team 16660 BioBuzz — Complete Operations Manual

> **WHO THIS IS FOR**
> This manual is written so that someone who has **never seen this codebase**
> can set the robot up, flash it, run it, and fix it when it breaks — without
> calling the lead developer.
>
> **Start here:** Chapter 0 is a 10-minute path to a working robot. Every other
> chapter is reference detail you look up when something is wrong.
>
> **Chapter index:**
> 0. [Start Here — First 10 Minutes](#chapter-0)
> 1. [Match Day Runbook](#chapter-1)
> 2. [Computer Setup from a Fresh Clone](#chapter-2)
> 3. [Flashing the Robot](#chapter-3)
> 4. [Pre-Flight Check — Is the Robot Ready?](#chapter-4)
> 5. [Hardware Configuration & Wiring](#chapter-5)
> 6. [Drivetrain: Directions, Odometry, Tuning](#chapter-6)
> 7. [The IMU, the Dead Wheels, and What Breaks Gracefully](#chapter-7)
> 8. [Shooter & Flywheel](#chapter-8)
> 9. [Vision](#chapter-9)
> 10. [Autonomous](#chapter-10)
> 11. [TeleOp](#chapter-11)
> 12. [Troubleshooting — Every Symptom](#chapter-12)
> 13. [Advanced: Extending the Codebase](#chapter-13)
> 14. [Appendix: All Constants](#chapter-14)
> 15. [Glossary](#chapter-15)

---

# Chapter 0: Start Here — First 10 Minutes

## 0.1 The situation and the plan

The robot may be untested. The plan is to **make the robot work without you**
by:

1. Getting the code onto the robot (Chapter 3).
2. Proving every subsystem with the **PRE-FLIGHT CHECK** OpMode (Chapter 4).
3. Fixing only what the pre-flight check flags as broken (Chapter 12).

Do NOT start by tuning. Tuning is the LAST step, and it is optional.

## 0.2 The three commands that matter

| Goal | Command |
|------|---------|
| Set up the project on a new computer | `bash scripts/setup-mac.sh` (Windows: `powershell -ExecutionPolicy Bypass -File scripts\setup-windows.ps1`) |
| Flash the robot | `bash scripts/deploy_to_hub.sh` (Windows: `powershell -ExecutionPolicy Bypass -File scripts\deploy_to_hub.ps1`) |
| Check the robot is ready | Run the **PRE-FLIGHT CHECK** OpMode on the Driver Station |

## 0.3 If the robot has never had this code: do this, in order

**Step 1 — Plug in the robot and flash it (5 min)**

```
Plug the Control Hub into the laptop with a USB-C DATA cable.
bash scripts/deploy_to_hub.sh
```

The script prints **FLASH COMPLETE AND VERIFIED** when it is done. If it
prints anything else, go to Chapter 3.

**Step 2 — Run the pre-flight check (2 min)**

On the Driver Station:
1. Press **INIT** (the OpMode list appears).
2. Scroll to the **Test** group.
3. Select **PRE-FLIGHT CHECK**.
4. Press **START**.

The robot prints a report of PASS / WARN / FAIL for every subsystem, and a
final line that says **READY** or **NOT READY**.

**Step 3 — Fix only the FAILs (varies)**

Each FAIL names the problem. Go to **Chapter 12 (Troubleshooting)** and look
up the exact symptom. The FAIL lines tell you which chapter.

**Step 4 — Prove the robot drives (2 min)**

Run **Drive Direction Check** (Test group):
- Press **A** → the robot should drive forward 50 cm.
- Hold **X** → the pose should read about x = +19.7 inches.

If it drives sideways or spins, go to **Chapter 6.2**.

**Step 5 — Prove the shooter works (2 min)**

Run **Flywheel Direction Test** (Test group):
- Hold **X** → both flywheels spin.
- They must **counter-rotate** (one clockwise, one counter-clockwise).

**Step 6 — Drive it**

Run **BioBuzz TeleOp**. The left stick drives, the right stick turns.

## 0.4 What "READY" means, and what to do when it is not READY

The pre-flight check separates problems into two classes:

| Class | Meaning | What to do |
|-------|---------|------------|
| **FAIL** | Something is broken. The robot cannot do that thing. | Fix it before the match (Chapter 12) |
| **WARN** | Something is missing, but the robot still works without it. | Usually fine to compete |

**A missing camera is a WARN, not a FAIL.** The robot drives and shoots without
cameras; it just cannot aim at a hive. Missing dead wheels is also a WARN — the
robot falls back to drive-motor odometry.

**A missing IMU is a WARN.** The robot still drives; its heading just comes
from the wheel encoders instead of the IMU, so it drifts more.

So a robot with no cameras and no dead wheels can still compete in teleop.
That is the point of the graceful-degradation design.

## 0.5 If everything is broken and you do not know why

Run these three in order and read the output:

```bash
# 1. Does the code even build?
./gradlew :TeamCode:assembleDebug

# 2. Is the robot connected and can we talk to it?
bash scripts/deploy_to_hub.sh

# 3. What does the robot think about itself?
#    (run PRE-FLIGHT CHECK on the Driver Station)
```

Then read Chapter 12. It is organized by symptom, so find your symptom and
follow it.

---

# Chapter 1: Match Day Runbook

Print this page. Follow it top to bottom.

## 1.1 Before you leave for the venue

- [ ] Laptop charged. Bring the **USB-C data cable** (not a charge-only cable).
- [ ] Laptop has the project: `bash scripts/setup-mac.sh` (or Windows script).
- [ ] Robot battery charged, and a **spare battery** packed.
- [ ] Know how to get the robot onto the field (wheelie cart, not carrying it).

## 1.2 At the venue, before the robot leaves your hands

1. **Flash it.** `bash scripts/deploy_to_hub.sh`
   - You must see `FLASH COMPLETE AND VERIFIED`.
   - If it says the app is NOT running, go to Chapter 3.4.
2. **Pre-flight.** Run **PRE-FLIGHT CHECK** → press START.
   - You must see `*** READY ***` or only WARNs you accept.
   - Any FAIL: fix now (Chapter 12) or note it and check whether it affects
     the mode you are running.
3. **Drive test.** Run **Drive Direction Check**, press **A**. It must drive
   straight forward 50 cm.
4. **Shooter test.** Run **Flywheel Direction Test**, hold **X**. Both wheels
   spin and counter-rotate.
5. **Vision test** (only if the robot will run autonomous): run
   **Vision Test OpMode** and confirm the ball blobs appear.
6. **Autonomous test** (only if running auto): run **Simple 50cm Out-Back**.
   It must drive forward 50 cm and return.

## 1.3 Between matches

1. Put the robot on the charger.
2. **Do not change code** unless something is broken. A code change at a
   competition is a risk with no upside.
3. If you changed a constant, note it — write it down.

## 1.4 If the robot stops working between matches

1. Run **PRE-FLIGHT CHECK** again. What changed? (battery low, a cable
   knocked loose, someone unplugged a motor)
2. Fix that one thing.
3. Re-run pre-flight.
4. If it passes pre-flight but still misbehaves, go to Chapter 12.

## 1.5 The two OpModes you will actually use

| Situation | OpMode |
|-----------|--------|
| Driver-controlled match | **BioBuzz TeleOp** (TeleOp group) |
| Autonomous match | **BioBuzz Autonomous** (Auto group) |

Everything else is a test or diagnostic.

## 1.6 Buttons you will use in a match

| Button | In TeleOp | Meaning |
|--------|-----------|---------|
| Left stick | Drive | Push forward to drive forward, right to strafe right |
| Right stick X | Turn | Right to turn right |
| **A** | Intake in | Pick up a ball |
| **B** | Intake out | Eject a ball |
| **X** | Fire one ball | Only with the loader |
| **RT** | Spin up + fire | Hold to spin the flywheel to the right speed |
| **LB** | Ball assist | Strafes toward the nearest ball |
| **RB** | Aim assist | Turns toward the hive and auto-feeds |
| **Y** | Drive scheme | Switches between normal and raw-stick driving |
| **D-pad ←/→** | (during INIT) | Red / Blue alliance |

---

# Chapter 2: Computer Setup from a Fresh Clone

## 2.1 Why this chapter exists

The single most common reason "the code only works on my computer" is
`local.properties`. It holds the path to the Android SDK, it is specific to
each computer, and it is **not in git** (on purpose — see `.gitignore`).
Every new computer needs it generated. The setup scripts do that for you.

## 2.2 What you need

| Tool | Version | Notes |
|------|---------|-------|
| JDK | 17 or newer | The build refuses to run on older |
| Android SDK | Platform 30, Build-Tools 30.0.3, Platform-Tools | Platform-Tools is what talks to the robot |
| Git | any recent | Only needed the first time |
| The project | — | From `git clone` |

## 2.3 macOS

```bash
git clone <repo-url>
cd 16660-BioBuzz
bash scripts/setup-mac.sh
```

The script:
1. Checks Java is 17+.
2. Finds your SDK (`~/Library/Android/sdk` is the normal place).
3. Writes `local.properties`.
4. Checks `adb` and the SDK components.
5. Builds and runs the tests.

**If the SDK is not found** it asks you for the path. Install Android Studio
from <https://developer.android.com/studio> if you do not have it, then let
the SDK Manager install the components listed above.

## 2.4 Windows

```powershell
git clone <repo-url>
cd 16660-BioBuzz
powershell -ExecutionPolicy Bypass -File scripts\setup-windows.ps1
```

If PowerShell refuses to run the script, that is the execution policy. The
`-ExecutionPolicy Bypass` flag above handles it. If you get "cannot find
gradlew", make sure you are in the project folder (the folder containing
`settings.gradle`).

**Windows note:** the SDK path contains a colon (`C:\Users\...`), which Java
properties files treat specially. The script escapes it automatically
(`C\:\\Users\\...`). If you write `local.properties` by hand, escape the
colon or the build fails with a confusing "SDK location not found".

## 2.5 Linux

```bash
git clone <repo-url>
cd 16660-BioBuzz
bash scripts/setup-linux.sh
```

Extra on Linux: you may need USB permission rules for the Control Hub:

```bash
echo 'SUBSYSTEM=="usb", ATTR{idVendor}=="18d1", MODE="0666"' \
  | sudo tee /etc/udev/rules.d/99-ftc.rules
sudo udevadm control --reload-rules && sudo udevadm trigger
```

## 2.6 Verifying the setup worked

```bash
./gradlew :TeamCode:assembleDebug :TeamCode:test
```

You want:
- `BUILD SUCCESSFUL`
- 50 tests, 0 failures.

If the build fails, the first `e:` line in the output is the actual error.
Everything after it is noise.

## 2.7 Common setup problems

| Error message | Cause | Fix |
|---------------|-------|-----|
| `SDK location not found` | `local.properties` missing or wrong | Run the setup script |
| `Unsupported class file major version` | Java too old | Install JDK 17+, set `JAVA_HOME` |
| `Could not resolve all artifacts` | No network, or a proxy | Check network; retry |
| `Permission denied` on `gradlew` | Clone lost the executable bit | `chmod +x gradlew` |
| `adb: command not found` | Platform-Tools not installed or not on PATH | The scripts use the SDK path directly, so this only affects manual `adb` use |
| `Gradle daemon journal locked` | Two Gradle runs at once | `./gradlew --stop`, then retry |

---

# Chapter 3: Flashing the Robot

## 3.1 What "flashing" means

The robot runs an Android app. "Flashing" means building that app as an APK
and installing it on the Control Hub over USB. There is no separate
firmware-flashing step for your code.

## 3.2 The normal way

```bash
bash scripts/deploy_to_hub.sh
```

Windows:

```powershell
powershell -ExecutionPolicy Bypass -File scripts\deploy_to_hub.ps1
```

If the robot is not plugged in yet, use `--wait` (or `-Wait`) and it will wait
for it.

What the script does, and what each step means:

| Step | What it does | If it fails |
|------|--------------|-------------|
| 1 | Finds `adb` from `local.properties` | Run the setup script |
| 2 | Builds the APK | Read the first `e:` error |
| 3 | Waits for the Control Hub | Check the cable (see 3.3) |
| 4 | `adb install -r` | See 3.4 |
| 5 | Launches the Robot Controller app | See 3.4 |
| 6 | **Verifies the app is running and did not crash** | See 3.4 |

**Step 6 is the one that matters.** `adb install` printing "Success" only means
the file was copied. If the app crashes the instant it starts, the robot is
running the OLD code (or nothing) and you will not know. The script checks the
process ID and scans logcat for crashes.

## 3.3 The cable (the #1 cause of "it won't connect")

- Use a **USB-C DATA** cable. Many cheap USB-C cables are **charge only** and
  carry no data. This looks exactly like a working cable.
- Plug directly into the Control Hub, not through a powered hub.
- Try another USB port, and another cable.
- On the robot, the Control Hub has two USB-C ports; either works.

Check what the computer sees:

```bash
adb devices -l
```

A connected Control Hub looks like:

```
List of devices attached
034318a27b8fe61c   device usb:17825792X product:ch_v1_box model:Control_Hub_v1_0
```

If the list is empty, it is a cable, port, or power problem — not software.

## 3.4 When the flash "succeeds" but the robot does not work

The script prints "The app is NOT running. Something crashed on launch." and
dumps the crash. Get more detail:

```bash
adb logcat -d -t 400 | grep -iE "FATAL EXCEPTION|AndroidRuntime"
```

Common causes of a crash on launch:

| Crash text | Cause | Fix |
|-----------|-------|-----|
| `HardwareMap.get() throws` / "device not found" | The Robot Configuration is missing a device the code expects | Chapter 12.1 |
| `ClassNotFoundException` / `NoSuchMethodError` | A stale APK, or a partial library mismatch | Re-flash; if it persists, `adb uninstall com.qualcomm.ftcrobotcontroller` then flash again |
| `NullPointerException` in a vision class | A camera is configured but broken | Chapter 9 |

## 3.5 Flashing by hand (if the script is unavailable)

```bash
# 1. Build
./gradlew :TeamCode:assembleDebug

# 2. Find adb (it is usually NOT on your PATH)
export PATH="$HOME/Library/Android/sdk/platform-tools:$PATH"   # macOS
# Windows PowerShell:
#   $env:PATH += ";$env:LOCALAPPDATA\Android\Sdk\platform-tools"

# 3. Check the connection
adb devices -l

# 4. Install
adb install -r TeamCode/build/outputs/apk/debug/TeamCode-debug.apk

# 5. Launch
adb shell am force-stop com.qualcomm.ftcrobotcontroller
adb shell am start -n \
  com.qualcomm.ftcrobotcontroller/org.firstinspires.ftc.robotcontroller.internal.FtcRobotControllerActivity

# 6. VERIFY
adb shell pidof com.qualcomm.ftcrobotcontroller      # must print a number
adb logcat -d -t 400 | grep -iE "FATAL EXCEPTION|AndroidRuntime"
```

The package is `com.qualcomm.ftcrobotcontroller`. It is NOT
`org.firstinspires.ftc.teamcode` — that is the library package inside the app
and is not installable.

## 3.6 Reading the robot's saved configuration

The Control Hub stores its robot configurations as XML. You can read them
without taking the robot apart:

```bash
adb shell 'ls /sdcard/FIRST/*.xml'
adb shell 'cat /sdcard/FIRST/<name>.xml'
```

This is the authoritative list of what is plugged in and on which port. Use it
when the pre-flight check says a device is missing — it tells you the name the
config uses, which may differ from the name in the code (Chapter 12.1).

---

# Chapter 4: Pre-Flight Check — Is the Robot Ready?

## 4.1 What it does

**PRE-FLIGHT CHECK** (Test group) inspects every subsystem without driving the
robot and prints a PASS / WARN / FAIL report.

It checks:
1. **Drive motors** — all four present, encoders reading, polarity table printed
2. **IMU** — present, initializes, yaw zeroed
3. **Flywheels** — both spin up to a test speed and report RPM
4. **Intake and loader** — configured, loader homes
5. **Cameras** — present or not
6. **Battery** — voltage under load

It does **not** drive the robot. It is safe to run with the robot on the
stand, wheels off the ground, or on the field.

## 4.2 Running it

1. Driver Station → **INIT** → scroll to the **Test** group.
2. Select **PRE-FLIGHT CHECK**.
3. Put the robot on a **stand** (wheels off the ground) for the flywheel test.
4. Press **START**.
5. Read the report.

## 4.3 Reading the report

```
===== PRE-FLIGHT REPORT =====
PASS Drive motors: all 4 present, encoders reading (0 0 0 0)
PASS Drive polarity: FL FWD FR REV BL FWD BR REV  (mirror check: FL/BL same, FR/BR same)
PASS IMU: initialized, yaw zeroed (now 0.0 deg)
PASS Flywheels: both spin (L=603 R=601 RPM)
WARN Cameras: missing FRONT — driving and shooting still work, aiming does not
WARN Dead wheels: not configured — using drive-motor odometry
PASS Battery: 12.8 V
================================
FAIL=0  WARN=2
*** READY ***
```

| Line | Meaning |
|------|---------|
| `FAIL n` | n things are broken |
| `WARN n` | n things are missing but survivable |
| `*** READY ***` | No FAILs. The robot can compete. |
| `*** NOT READY ***` | At least one FAIL. Read the FAIL lines. |

## 4.4 What each FAIL means

| FAIL | What broke | Where to fix it |
|------|-----------|-----------------|
| `Drive motors: NONE ... configured` | No drive motors in the Robot Configuration | Chapter 12.1 |
| `Drive motors: missing: ...` | Some drive motors are missing | Chapter 12.1 |
| `Drive motors: no encoder reading from: ...` | A drive encoder is dead or unplugged | Chapter 12.2 |
| `IMU: drive construction failed` | The IMU is breaking the whole drive | Chapter 7.2 |
| `Flywheels: did not reach N RPM` | A flywheel is not spinning (or is loaded) | Chapter 8.4 |
| `Flywheels: error` | Flywheel configuration problem | Chapter 8.4 |
| `Battery: X V — below Y V` | Battery too flat | Chapter 12.11 |

## 4.5 What each WARN means

| WARN | Consequence | OK to compete? |
|------|-------------|----------------|
| `Cameras: missing ...` | No aiming, no ball detection, no autonomous | Teleop: yes. Autonomous: no |
| `Dead wheels: not configured` | Odometry uses drive wheels; it slips | Yes, but odometry is less accurate |
| `IMU: not configured` | Heading from encoders only; drifts | Yes, but field-centric and auto-aim suffer |
| `IMU: present but FAILED to initialize` | Same as above | Yes; check the cable if you want the IMU |
| `Loader: not configured` | Cannot feed balls into the flywheel | Only if you do not need to shoot |
| `Intake: not configured` | Cannot collect balls | Only if you do not need to collect |
| `Battery: could not read the voltage sensor` | Unknown battery state | Check the battery by hand |

## 4.6 If the pre-flight check itself crashes

That means something in the Robot Configuration is actively broken (not just
missing). Get the crash:

```bash
adb logcat -d -t 400 | grep -iE "FATAL EXCEPTION|AndroidRuntime"
```

Then Chapter 12.1.

---

# Chapter 5: Hardware Configuration & Wiring

## 5.1 The Robot Configuration (on the Driver Station)

The robot's hardware lives in a **Robot Configuration** file on the Control
Hub, edited from the Driver Station: **Program & Manage → Robot Controller →
Configure Robot**. This is separate from the code — the code asks for devices
**by name**.

| Device | Name in the config | Where | Notes |
|--------|-------------------|-------|-------|
| Drive motors | `frontleft`, `frontright`, `backleft`, `backright` | Control Hub 0–3 | 4× goBILDA 5203-2402-0019, 312 RPM |
| Flywheels | `flywheelleft`, `flywheelright` | Expansion Hub 1–2 | 6000 RPM |
| Intake | `intake` | Expansion Hub 0 | 5202 series |
| Loader | `loader` (servo) | Control Hub servo port 0 | Continuous rotation |
| IMU | `imu` | Built into the Control Hub | Never unplug this |
| Cameras | `Webcam High`, `Webcam Ground` | USB | UP + FRONT |
| Odometry pods | `backleftodo`, `backrightodo`, `frontodo` | Expansion Hub | **Optional** — see Chapter 7 |

**Verified port map on the competition bot** (read from the hub over adb):

| Hub | Port | Device |
|-----|------|--------|
| Control Hub | 0 | `backright` |
| Control Hub | 1 | `backleft` |
| Control Hub | 2 | `frontright` |
| Control Hub | 3 | `frontleft` |
| Control Hub | servo 0 | `loader` |
| Control Hub | IMU | `imu` |
| Expansion Hub 2 | 0 | `intake` |
| Expansion Hub 2 | 1 | `flywheelleft` |
| Expansion Hub 2 | 2 | `flywheelright` |

**If a name in the config does not match the name in the code, the device is
simply not found.** The code never crashes for a missing device — it reports it
and continues. See Chapter 12.1.

## 5.2 The four files that matter

If you change anything, it will be in one of these:

| File | Owns | Change it when |
|------|------|----------------|
| `driveconfig.kt` | Drive motor polarity + mixing matrix | A wheel spins the wrong way |
| `drivetrainspec.kt` | Physical measurements (wheel size, RPM, geometry) | You measured something new |
| `globals.kt` | Everything else (HSV, timeouts, gains, flywheel direction) | Game-day tweaks |
| `PreFlightCheck.kt` | The pre-flight report | You add a subsystem to check |

## 5.3 Motor direction — the single place to change it

**All drive-motor direction is in `driveconfig.kt`:**

```kotlin
const val FL_REVERSED = false   // frontleft
const val FR_REVERSED = true    // frontright
const val BL_REVERSED = false   // backleft
const val BR_REVERSED = true    // backright
```

These are the values proven on this robot (they match JJ's `TestTeleOP`).

### The rule people get wrong

On a **mecanum** the two sides are mirrored, so:

- **FL and BL spin one rotational way.**
- **FR and BR spin the OTHER way.**
- Commanding +1 to all four drives the robot **FORWARD**.

"Make all four wheels spin the same way" is a **wiring bug** on a mecanum. Do
not do it.

### How to fix a wrong wheel

1. Edit the one flag in `driveconfig.kt`.
2. `bash scripts/deploy_to_hub.sh`
3. Run **Wheel Direction Test** and confirm D-pad UP drives straight.

**Never rewire the hub to fix a direction.** One boolean is a 30-second fix;
rewiring is 30 minutes and can introduce a second problem.

## 5.4 Flywheel spin direction

In `globals.kt`:

```kotlin
const val FLYWHEEL_LEFT_REVERSED = false
const val FLYWHEEL_RIGHT_REVERSED = true
```

Full swap procedure: **Chapter 8.3**.

## 5.5 Dead wheel (odometry pod) names

In `globals.kt`:

```kotlin
const val DEAD_WHEEL_PAR0_NAME = "backleftodo"
const val DEAD_WHEEL_PAR1_NAME = "backrightodo"
const val DEAD_WHEEL_PERP_NAME = "frontodo"
```

**These pods are optional.** If they are not in the Robot Configuration, the
robot automatically falls back to drive-motor odometry. See Chapter 7.

## 5.6 IMU orientation

The Control Hub is mounted on the **left side, pointing inward**:

| Property | Value |
|----------|-------|
| Logo faces | **RIGHT** |
| USB ports face | **UP** |

These live in `roadrunner/MecanumDrive.kt` (`Params`) and are live-tunable via
FTC Dashboard. **Wrong values = heading drift** — the robot turns the wrong way
when told to go straight, and field-centric driving behaves oddly.

If the hub is remounted, change these two values and re-flash.

---

# Chapter 6: Drivetrain — Directions, Odometry, Tuning

## 6.1 The drive hardware

| Property | Value | Where it comes from |
|----------|-------|--------------------|
| Motor | goBILDA 5203-2402-0019 | Spec sheet |
| Free speed | 312 RPM | Spec sheet |
| Gearbox | 19.2:1 | Spec sheet |
| Encoder | 28 counts/rev (motor shaft) | Spec sheet |
| Wheel diameter | 10.4 cm | **Measured** |
| Track width | 42 cm (left↔right wheel centers) | **Measured** |
| Wheelbase | 32 cm (front↔rear axle) | **Measured** |

**537.6 counts per wheel revolution** = 28 × 19.2. Everything else is derived
from that in `drivetrainspec.kt`. You never edit a derived constant.

## 6.2 Direction check — do this before anything else

Run **Wheel Direction Test** (Test group):

| Hold | Expect |
|------|--------|
| A | Only front-left spins |
| B | Only front-right spins |
| X | Only back-left spins |
| Y | Only back-right spins |
| D-pad UP | **The robot drives straight forward** |

The telemetry also shows each wheel's `dir=` (its polarity flag) and `enc=`
(its encoder position).

### If D-pad UP makes the robot STRAFE

One or two wheels are the wrong way. Look at the polarity printed on screen
and compare with the physical mounting: **FL/BL must be the same, FR/BR must
be the same, and the two pairs must be opposite.**

Fix: flip the wrong flag in `driveconfig.kt`, re-flash, re-test.

### If D-pad UP makes the robot SPIN IN PLACE

All four are wrong in the same direction, or two are flipped. Same fix.

### If the WRONG wheel moves

The hub config name does not match the code. Check the config over adb
(Chapter 3.6) and fix the name in `globals.kt`.

## 6.3 Odometry — where the robot thinks it is

The robot tracks its own position so it can drive to a target and so
autonomous can aim.

| Configuration | How position is computed | Quality |
|--------------|------------------------|---------|
| 3 dead wheels configured | Wheel odometry + IMU heading | **Good** — no slip |
| Dead wheels missing | Drive-wheel odometry + IMU heading | Fair — slips under acceleration |

**If dead wheels are missing, the robot still works.** This is deliberate. See
Chapter 7.

### Checking the odometry

Run **Drive Direction Check** (Test group):
- Press **A** → drives forward 50 cm.
- Hold **X** → the pose should read **x ≈ +19.7 inches**, **y ≈ 0**.

| What you see | Meaning |
|--------------|---------|
| x = +19.7, y = 0 | Perfect |
| x = +19.7, y = 5 | The robot drifted sideways — mecanum slip, or a wheel is binding |
| x = -19.7 | The robot drove backward (check Chapter 6.2) |
| x = 0 | The pose is not updating — the localizer is broken |

## 6.4 Road Runner tuning (optional — the robot drives without it)

The robot's trajectory controller has settings that **already have working
defaults derived from the hardware**. You do not need to tune them to make the
robot drive. Tune only to make it drive *better*.

Open FTC Dashboard at `http://<rc-ip>:8080` → **Config** →
`MecanumDrive$Params`.

| Setting | Default | Meaning |
|---------|---------|---------|
| `kS` | 0.6 | Static friction, in **volts** |
| `kV` | derived | Velocity feedforward, **volts per tick/sec** |
| `kA` | 0.0 | Acceleration feedforward |
| `maxWheelVel` | 50.0 | Max wheel speed, in/s |
| `axialGain` / `lateralGain` | 2.0 | in/s per inch of position error |
| `headingGain` | 2.0 | rad/s per radian of heading error |
| `axialVelGain` / `lateralVelGain` | 0.5 | in/s per in/s of velocity error |
| `headingVelGain` | 0.5 | rad/s per rad/s of heading error |

**The rule:** if the robot **undershoots** (does not reach the target), raise
`kV` slightly. If it **overshoots or oscillates**, lower it. If it converges
slowly without oscillating, raise the position gains a little.

**Do not set `kS`, `kV` or the gains to zero.** A zero feedforward means
Road Runner commands **no motor power at all** and every trajectory move
silently does nothing while the code looks fine. That is the single most
expensive mistake in this codebase, and it is why the defaults are derived
rather than zero.

## 6.5 How the mixing works

`DriveConfig.mix()` is the ONE mixing matrix, used by both the teleop and the
autonomous:

```
FL = forward + strafe + turn
FR = forward - strafe - turn
BL = forward - strafe + turn
BR = forward + strafe - turn
```

Scaled by `1 / (|forward| + |strafe| + |turn|)` so no wheel is ever asked for
more than full power.

Sign convention: **forward = +**, **strafe right = +**, **turn right
(clockwise) = +**.

---

# Chapter 7: The IMU, the Dead Wheels, and What Breaks Gracefully

## 7.1 The design rule

**Nothing optional may take the robot down.** The robot is expected to work
with cameras missing, dead wheels missing, the IMU missing or broken, the
loader missing, or the intake missing. Each of those is a **WARN** in the
pre-flight check, not a crash.

| Missing | What the robot does instead |
|---------|---------------------------|
| Cameras | Drives and shoots without aiming. No ball detection. |
| Dead wheels | Uses drive-motor odometry (more slip, still works) |
| IMU | Heading from wheel encoders only (drifts more) |
| One flywheel | Spins one wheel; cannot launch straight |
| Loader | Cannot feed balls into the flywheel |
| Intake | Cannot collect balls |

## 7.2 The IMU is safe now — here is why that mattered

Road Runner's `LazyHardwareMapImu.get()` calls `imu.initialize()`, which
**throws** if the IMU is present in the config but unhealthy (loose cable,
wrong orientation, hub still booting). The localizers call `get()` from their
constructor, so a lazy IMU meant **a bad IMU could crash every OpMode** —
including the bench tests you would use to diagnose it.

The IMU is now initialized eagerly inside a `try/catch`:

- If it initializes, its **yaw is zeroed**, so heading reads 0 for the robot's
  current orientation.
- If it fails, the robot falls back to encoder-only heading and the pre-flight
  check reports a WARN with the reason.

### Why zeroing the yaw matters

The IMU's yaw is an arbitrary angle left over from whenever the hub was
powered on. Without a zero:

- Field-centric driving would be off by that unknown angle.
- An autonomous turn-to-heading would be off by that unknown angle.

Zeroing at the start of every OpMode makes heading meaningful and repeatable.

### If the IMU reports a WARN

1. Check the Control Hub is seated and powered.
2. Run **PRE-FLIGHT CHECK** — it prints the IMU error message.
3. The robot still drives. Only field-centric and auto-aim suffer.

## 7.3 The IMU as a backup for the dead wheels

The dead wheels measure position; the IMU measures heading. They are
independent, so:

- **Dead wheels present, IMU broken:** position is still good (the pods do not
  slip), but heading drifts. The robot drives straight-ish but turns
  accumulate error.
- **Dead wheels missing, IMU working:** heading is perfect, position slides.
  This is the common case and it is fine.

That independence is why both are optional: whichever one is present carries
the other.

## 7.4 If the robot's heading is wrong

Run **Drive Direction Check**, hold **X**, and rotate the robot 90°
counter-clockwise by hand.

| Reading | Meaning | Fix |
|---------|---------|-----|
| Heading goes **+90°** | IMU orientation is right | Nothing |
| Heading goes **−90°** | IMU orientation is wrong | Chapter 5.6 — swap logo/USB directions |
| Heading does not change | IMU is not being read | Chapter 12.3 |

---

# Chapter 8: Shooter & Flywheel

## 8.1 The shooter hardware

| Property | Value |
|----------|-------|
| Motors | 2× goBILDA 5203-2402-0001, 6000 RPM |
| Configuration | One wheel each side of the ball, **counter-rotating** |
| Feed | `loader` servo meters one ball at a time |
| Launch | Ball squeezed between the wheels and thrown forward |

## 8.2 Checking the flywheels

Run **Flywheel Direction Test** (Test group). Put the robot on a **stand** —
the wheels spin fast.

| Hold | Expect |
|------|--------|
| A | Only the LEFT wheel spins |
| B | Only the RIGHT wheel spins |
| X | BOTH spin — and they **counter-rotate** |
| Y | Everything stops |

Telemetry shows commanded RPM and measured RPM for each side.

| Symptom | Cause | Fix |
|---------|-------|-----|
| Both wheels spin the same way | One direction flag is wrong | Chapter 8.3 |
| Measured RPM is negative for one side | That side's direction flag is flipped | Chapter 8.3 |
| A wheel does not reach the commanded RPM | Wheel is jammed, or the battery is low | Chapter 12.11 |
| Neither flywheel is configured | Wiring/config problem | Chapter 12.1 |

## 8.3 How to swap the flywheel spin direction

Two constants in `globals.kt`:

```kotlin
const val FLYWHEEL_LEFT_REVERSED = false
const val FLYWHEEL_RIGHT_REVERSED = true
```

| Symptom | Fix |
|---------|-----|
| **The ball comes out the BACK** (toward the robot) | Flip **BOTH** constants |
| **Only one wheel spins the wrong way** | Flip only that wheel's constant |
| **Wheels counter-rotate but the ball squirts sideways** | The two motors are **swapped left-for-right** in the hub config — swap the names there, or swap which constant drives which motor |
| **Measured RPM is negative for a wheel that looks normal** | That wheel's flag is flipped |

Procedure:

1. Edit `globals.kt`.
2. `bash scripts/deploy_to_hub.sh`
3. Run **Flywheel Direction Test** again. Confirm the wheels still counter-rotate
   and the measured RPMs are positive.

## 8.4 Why the shooter RPM is not always full speed

A full-power shot from 100 cm sails straight past the hive. The code picks
the RPM from the measured calibration curve if there is one, and otherwise
estimates it from projectile physics:

```
A ball squeezed between two flywheels leaves at roughly the flywheel's
surface speed. At a 45° launch, range d = v²/g, so:

    v     = sqrt(d * g)
    omega = v / (r * efficiency)
    RPM   = omega * 60 / (2π)
```

Constants in `globals.kt`:

| Constant | Default | Meaning |
|----------|---------|---------|
| `SHOOTER_FLYWHEEL_RADIUS_CM` | 3.8 | **Measure** the radius of ONE flywheel |
| `SHOOTER_TRANSFER_EFFICIENCY` | 0.55 | Fraction of surface speed the ball gets |
| `FLYWHEEL_MIN_FIRING_RPM` | 1000 | Never command less than this when firing |

Estimated RPM by distance with the defaults:

| Distance | 80 cm | 100 cm | 150 cm | 200 cm | 250 cm |
|----------|-------|--------|--------|--------|--------|
| RPM | 1279 | 1430 | 1752 | 2023 | 2262 |

The estimate is deliberately a little HIGH, so the robot **undershoots** rather
than overshoots — undershooting still scores at close range, overshooting sails
past.

### Measuring the real curve (10 minutes, once)

1. Run **Flywheel Calibration** (Test group).
2. **A** toggles the flywheel spin; **X** fires ONE ball.
3. **D-pad ↑/↓** picks the RPM step; **D-pad ←/→** fine-tunes ±25.
4. Film each shot side-on, with two tape marks a known distance apart in frame.
5. Analyze:
   ```bash
   cd tools && pip install -r requirements.txt
   python3 flywheel_analyzer.py --video shot_6000rpm_200cm.mp4 --rpm 6000 --distance-cm 200
   ```
   Click **1** = launch point, **2** and **3** = the tape marks, **c** confirms.
6. Paste the measured distances into `globals.kt`:
   ```kotlin
   var SHOOTER_CALIBRATION_DISTANCE_CM = doubleArrayOf(
       320.0,   // measured at 6000 RPM
       270.0,   // 5000 RPM
       225.0,   // 4000 RPM
       185.0,   // 3000 RPM
       150.0,   // 2000 RPM
       120.0,   // 1000 RPM
   )
   ```

Once two or more real points exist, `ShooterModel.isCalibrated()` is true and
the robot uses the measured curve.

## 8.5 The loader

The `loader` servo meters exactly one ball per stroke.

| Constant | Default | Meaning |
|----------|---------|---------|
| `LOADER_HOME_POSITION` | 0.0 | Gate closed |
| `LOADER_STROKE_END_POSITION` | 1.0 | One ball enters the flywheel |
| `LOADER_STROKE_MS` | 800 | Forward stroke duration |
| `LOADER_RETRACT_MS` | 600 | Return-to-home duration |

If balls jam in the loader, these two durations are the first things to
adjust (Chapter 12.9).

## 8.6 The shot confirmation

The robot knows a ball was fired because the flywheel RPM **dips** when the
ball passes through. Constants:

| Constant | Default | Meaning |
|----------|---------|---------|
| `SHOT_DIP_THRESHOLD_RPM` | 250 | Dip below baseline that counts as a shot |
| `SHOT_DETECT_WINDOW_MS` | 900 | How long after a stroke we watch |
| `SHOT_RECOVERY_TIMEOUT_MS` | 2500 | Wait for the wheels to recover |

If shots are not registering, lower `SHOT_DIP_THRESHOLD_RPM`. If false shots
are registering, raise it.

---

# Chapter 9: Vision

## 9.1 What vision is for

| Function | Needs |
|----------|-------|
| Find the hive (AprilTag) | UP camera + calibrated focal length |
| Find balls and flowers | FRONT camera + tuned HSV ranges |
| Correct the robot's pose | Both cameras + calibrated extrinsics |
| Aim assist (RB in teleop) | Both cameras |
| Autonomous collect and align | Both cameras |

**Missing cameras is a WARN, not a FAIL.** The robot drives and shoots without
them; it just cannot aim or find balls.

## 9.2 The two camera jobs

| Camera | Config name | Mount | Looks at |
|--------|-------------|-------|----------|
| UP | `Webcam High` | Angled up | AprilTags on the hive |
| FRONT | `Webcam Ground` | Forward/down | Balls and flowers on the floor |

If only one camera is present, the code runs with that one. If both are
missing, the vision stack is not built at all and the teleop says
`MANUAL ONLY`.

## 9.3 Calibrating the cameras (only if aiming does not work)

The camera focal length is a **measurement**, and the code has a placeholder
(`C270_FOCAL_LENGTH_PIXELS = 543`) until you do it.

1. Print an AprilTag that is **NOT** a BioBuzz hive tag (IDs 30–45 are hive
   tags; use ID 1 from the 36h11 family).
2. Measure the **black square** width in mm — not the white border.
3. Tape it flat, dead-center in front of one camera, facing it squarely.
4. Measure the lens-to-tag distance.
5. Run **Camera Calibration (C270)** (Test group).
   - **D-pad ↑/↓** = distance (+1 cm; hold **X** for 10 cm)
   - **D-pad ←/→** = tag width (+1 mm; hold **X** for 10 mm)
   - Hold **A** → averages 30 frames and shows a stable focal length
6. Write the value into `vision/VisionConfig.kt`:
   ```kotlin
   val UP_CAMERA = CameraIntrinsics(
       focalLengthPx = YOUR_MEASURED_VALUE,
       centerX = 400, centerY = 224,   // half of 800x448
       distortion = doubleArrayOf(0.0, 0.0, 0.0, 0.0, 0.0)
   )
   val FRONT_CAMERA = CameraIntrinsics(
       focalLengthPx = YOUR_MEASURED_VALUE,
       centerX = 320, centerY = 240,   // half of 640x480
       distortion = doubleArrayOf(0.0, 0.0, 0.0, 0.0, 0.0)
   )
   ```

Sanity checks on the calibration screen: "dist with cfg" should match your tape
measure, and the horizontal FOV should land around 55–60° for a C270.

## 9.4 Tuning the ball colors

Run **Vision Test OpMode** (Test group):

```
Bumpers (LB/RB)     → Cycle target: Pollen, Nectar Blue, Nectar Red, Nectar Red Wrap, Flower, Robot
D-pad ↑/↓           → Select H/S/V Min/Max
D-pad ←/→           → Adjust value (-1)
X + D-pad ←/→       → Adjust value (-10 / +10)
A                   → Toggle the calibration overlay
```

1. Put a game piece under the FRONT camera in competition lighting.
2. Adjust until the overlay box hugs the piece tightly with no noise.
3. Press **A** to confirm the overlay matches the piece.
4. Copy the values from telemetry into the `globals.kt` HSV constants.

### Current HSV defaults

```kotlin
POLLEN_HSV_RANGE          = intArrayOf(14, 35, 100, 255, 100, 255)
NECTAR_BLUE_HSV_RANGE     = intArrayOf(100, 140, 100, 255, 100, 255)
NECTAR_RED_HSV_RANGE      = intArrayOf(0, 12, 100, 255, 100, 255)
NECTAR_RED_WRAP_HSV_RANGE = intArrayOf(168, 180, 100, 255, 100, 255)
FLOWER_GREEN_HSV_RANGE    = intArrayOf(45, 80, 50, 255, 50, 255)
ROBOT_DARK_HSV_RANGE      = intArrayOf(0, 180, 0, 120, 0, 90)
```

There is also a desktop tool: `tools/hsv_calibrator.py` (GUI with sliders;
prints a paste-ready `ColorRange(...)` block).

## 9.5 Validating vision before autonomous

Run **Visual Localization Test** (Test group). It runs 6 checks:

1. **Stationary** — features stable
2. **Rotation** — heading tracks
3. **Translation** — position tracks
4. **Obstruction** — recovers when the view is blocked
5. **One camera blocked** — handles a single camera loss
6. **Both blocked** — goes quiet instead of lying

**All 6 must pass** before enabling pose fusion
(`FUSION_ENABLED_DEFAULT = false` by default — it stays off until you do).

## 9.6 Vision settings you can live-tune

FTC Dashboard → Config → `VisionTuning`:

| Parameter | Default | When to adjust |
|-----------|---------|----------------|
| `MAX_FEATURES` | 60 | More features = better tracking, slower |
| `DETECTION_QUALITY` | 0.05 | Lower = more features, more noise |
| `DETECTION_MIN_DISTANCE_PX` | 15.0 | Prevent feature clustering |
| `LK_WINDOW_SIZE` | 21 | Larger = more robust, slower |
| `LK_PYRAMID_LEVELS` | 3 | Pyramid levels |
| `LK_MAX_ERROR_PX` | 4.0 | Track rejection threshold |
| `LK_MIN_TRACKS` | 25 | Reacquire below this |
| `LK_REACQUIRE_INTERVAL_FRAMES` | 15 | Periodic re-detection |
| `RANSAC_ITERATIONS` | 40 | More = robust, slower |
| `RANSAC_INLIER_THRESHOLD_PX` | 3.0 | Pixel tolerance for an inlier |
| `RANSAC_MIN_INLIERS` | 12 | Minimum for a valid transform |
| `FUSION_ENABLED_DEFAULT` | false | Master gate for pose fusion |
| `FUSION_MIN_CONFIDENCE` | 0.35 | Below this, vision never moves the pose |
| `FUSION_MAX_GAIN` | 0.20 | Max correction per update |
| `FUSION_MAX_STEP_IN` | 0.30 | Max translational correction (inches) |
| `FUSION_MAX_STEP_DEG` | 2.0 | Max heading correction (degrees) |
| `FUSION_RESEED_MS` | 2500 | Re-seed the visual pose after loss |
| `GUIDANCE_K_FORWARD` | 0.10 | Vision guidance P gain (forward) |
| `GUIDANCE_K_STRAFE` | 0.10 | Vision guidance P gain (strafe) |
| `GUIDANCE_K_TURN` | 0.02 | Vision guidance P gain (turn) |
| `GUIDANCE_POS_TOLERANCE_IN` | 0.75 | Settle tolerance (inches) |
| `GUIDANCE_HEADING_TOLERANCE_DEG` | 3.0 | Settle tolerance (degrees) |
| `GUIDANCE_SETTLE_FRAMES` | 5 | Frames in tolerance to finish |
| `GUIDANCE_TIMEOUT_MS` | 4000 | Max guidance time |
| `GUIDANCE_CONFIDENCE_FLOOR` | 0.45 | Refuse to move below this confidence |

---

# Chapter 10: Autonomous

## 10.1 Before you run autonomous

Autonomous is the most fragile mode. Run this checklist **every time**:

- [ ] **Simple 50cm Out-Back** works (it is the simplest autonomous there is)
- [ ] **PRE-FLIGHT CHECK** shows no FAILs
- [ ] **Visual Localization Test** passes all 6
- [ ] **Flywheel Calibration** spins up cleanly
- [ ] Alliance is correct: **D-pad ← = RED**, **D-pad → = BLUE**, during INIT
- [ ] The robot starts where you expect it (the IMU yaw is zeroed at start,
      so heading 0 = the way the robot is facing when the OpMode starts)

## 10.2 How autonomous works

30 seconds, budgeted by a `PhaseClock`:

```
COLLECT → (FLOWER) → POLISH → ALIGN → FIRE → PARK
```

| Phase | What it does | Timeout |
|-------|--------------|---------|
| COLLECT | Intake on, chase the nearest visible ball, confirm the pickup | 9 s |
| FLOWER | Optionally pull a ball from a flower | 6 s |
| POLISH | One vision-guidance nudge to clean up pose drift (first cycle only) | — |
| ALIGN | Turn and drive until the hive is in the shooting window | 8 s |
| FIRE | Spin up, meter balls one at a time, confirm each shot | reserve 6 s |
| PARK | Back off 12 inches along the robot's heading | reserve 3 s |

The reserves mean the robot always parks, even if it ran out of time early.

## 10.3 The shooting window

The robot only fires when the hive is:

- between `SHOOT_MIN_RANGE_CM` (60) and the curve's reach (140 until measured)
- within `SHOOT_MAX_BEARING_DEG` (10) of straight ahead

If the hive is outside that window, the robot **drives closer** instead of
shooting. That is intended.

## 10.4 If autonomous does nothing at all

| Symptom | Cause | Fix |
|---------|-------|-----|
| Robot parks immediately, says "AUTONOMOUS HARDWARE NOT DETECTED" | Something in the required list is missing | Pre-flight check names it. Chapter 12.1 |
| Robot drives but never aims | Cameras missing or not seeing the hive | Chapter 9 |
| Robot aims but never shoots | Shooter not reaching speed, or the loader is missing | Chapter 8 |
| Robot shoots but the count is zero | The RPM dip is not being detected | Lower `SHOT_DIP_THRESHOLD_RPM` (Chapter 8.6) |
| Robot drives the wrong way | Motor polarity | Chapter 6.2 |

## 10.5 Tuning autonomous without a robot

Every autonomous timeout is a constant in `globals.kt`, so you can change the
behaviour of the robot from your laptop:

| Constant | Default | Effect |
|----------|---------|--------|
| `AUTONOMY_PERIOD_SEC` | 30.0 | Total autonomous length |
| `AUTONOMOUS_COLLECT_TIMEOUT_MS` | 9000 | How long it hunts for a ball |
| `FLOWER_PULL_TIMEOUT_MS` | 6000 | Flower attempt length |
| `MIN_CYCLE_REMAINING_SEC` | 14.0 | Minimum time to start another cycle |
| `AUTONOMY_PARK_RESERVE_SEC` | 3.0 | Time always kept for parking |
| `AUTONOMY_FIRE_RESERVE_SEC` | 6.0 | Time always kept for shooting |

**Make the robot more conservative at a competition**: raise
`MIN_CYCLE_REMAINING_SEC` so it stops starting cycles it cannot finish.

---

# Chapter 11: TeleOp

## 11.1 Controls

| Input | Action |
|-------|--------|
| Left stick Y | Forward / back |
| Left stick X | Strafe left / right |
| Right stick X | Turn |
| **Y** | Toggle drive scheme (normal ↔ raw-stick) |
| **Left stick button** | Toggle field-centric driving |
| **Right stick button** | Toggle pose hold |
| **A** | Intake in |
| **B** | Intake out |
| **X** | Fire one ball (uses the loader) |
| **RT** (hold) | Spin the flywheel to the right speed and fire |
| **LB** (hold) | Ball capture assist — strafe toward the nearest ball |
| **RB** (hold) | Shoot align assist — turn toward the hive and auto-feed |
| **D-pad ←/→** (during INIT) | Select RED / BLUE alliance |

## 11.2 The three drive modes

| Mode | What it does | When |
|------|--------------|------|
| **Robot-centric** (default) | Sticks move the robot relative to itself | Normal driving |
| **Field-centric** (L-stick button) | Push forward = drive away from you, always | Precise driving toward a goal |
| **Pose hold** (R-stick button) | When the sticks are centred, the robot holds its spot | Staying put |

## 11.3 The Y-button drive scheme

The **raw-stick** mode uses exactly the mixing and polarity from
`driveconfig.kt` — the same proven scheme the pushbot robot uses. Both modes
read the same table, so they can never disagree about which way the robot
faces.

Use raw-stick mode if the normal mode feels sluggish or the assists fight
you.

## 11.4 Assists

| Assist | Behaviour | Needs |
|--------|-----------|-------|
| **LB** ball capture | Strafes toward the nearest ball and stops at a standoff | FRONT camera |
| **RB** shoot align | Turns toward the hive, holds the shooting window, auto-feeds when the wheels are up to speed | UP camera, shooter, loader |

With no cameras the assists are simply absent — the robot still drives.

## 11.5 If the robot drives but feels wrong

| Symptom | Likely cause | Fix |
|---------|-------------|-----|
| Robot strafes when pushing forward | Motor polarity | Chapter 6.2 |
| Turning the wrong way | The turn sign, or an IMU orientation problem | Chapter 6.2, then 7.4 |
| Field-centric drifts | IMU heading is wrong | Chapter 7.4 |
| Robot creeps when the sticks are centred | Pose hold is on and the pose is drifting | Toggle pose hold off (**R**-stick button) |
| Robot drifts sideways when driving | Mecanum slip, or a binding wheel | Chapter 12.2 |
| Assists do nothing | Cameras missing or not seeing anything | Chapter 9 |
| Robot cuts power suddenly | Tilt guard tripped (robot tilted) | Chapter 12.10 |
| Robot shakes / oscillates | Too much pose hold, or bad odometry | Turn pose hold off; check odometry (6.3) |

## 11.6 Teleop tuning constants

| Constant | Default | Adjust if |
|----------|---------|-----------|
| `TELEOP_FIELD_CENTRIC_DEFAULT` | false | You want field-centric every match |
| `TELEOP_POSE_HOLD_DEFAULT` | true | You want the robot to hold position by default |
| `TELEOP_INPUT_DEADZONE` | 0.08 | The sticks drift at centre |
| `TELEOP_HOLD_POSITION_KP` | 0.06 | Pose hold is too weak / too strong |
| `TELEOP_HOLD_HEADING_KP` | 0.02 | Heading hold too weak / too strong |
| `TELEOP_HOLD_MAX_POWER` | 0.25 | Pose hold fights the driver |

---

# Chapter 12: Troubleshooting — Every Symptom

**Find your symptom. Follow that section. Do not skip ahead to tuning.**

## 12.1 "A device is missing" / "AUTONOMOUS HARDWARE NOT DETECTED"

**Symptom:** the pre-flight check reports a missing device; an OpMode says
`HardwareMap.get() throws`; a subsystem does nothing.

**Cause:** the Robot Configuration on the Control Hub does not contain a device
with the name the code asks for.

**Fix:**

1. Read the config from the robot:
   ```bash
   adb shell 'ls /sdcard/FIRST/*.xml'
   adb shell 'cat /sdcard/FIRST/<name>.xml'
   ```
2. Compare those names with the code's names (`globals.kt`):
   - `DrivetrainFrontLeft` = `"frontleft"` etc.
   - `FlywheelLeftMotor` = `"flywheelleft"`, `FlywheelRightMotor` = `"flywheelright"`
   - `IntakeMotor` = `"intake"`
   - `LoaderServoName` = `"loader"`
   - `RR_IMU_NAME` = `"imu"`
   - `UP_CAM_NAME` = `"Webcam High"`, `GROUND_CAM_NAME` = `"Webcam Ground"`
   - `DEAD_WHEEL_PAR0_NAME` = `"backleftodo"` etc.
3. If the names differ, either rename the device in the Driver Station
   (**Program & Manage → Robot Controller → Configure Robot**) or change the
   constant in `globals.kt` to match.
4. Re-flash and re-run the pre-flight check.

**Which devices are actually required?**

| Required for | Devices |
|--------------|---------|
| Driving at all | The 4 drive motors |
| Autonomous | Drive motors + cameras + IMU + 2 flywheels + intake + loader |
| Teleop | Drive motors (everything else is optional) |

## 12.2 "A wheel spins but the encoder does not move"

**Symptom:** the wheel spins, but its encoder number is frozen.

**Cause:** the encoder wire is loose, or the encoder is dead.

**Effect:** that wheel's closed-loop control does not work. Position moves and
velocity control on that wheel will not behave.

**Fix:**

1. Reseat the motor's encoder connector at the hub. Push it in firmly.
2. Try a different hub port for that motor.
3. If it is still frozen, the encoder is dead — the robot can still drive with
   raw power (that is exactly why **Wheel Direction Test** uses raw power),
   but position control will not work on that wheel.

## 12.3 "The IMU is broken" / "heading is wrong"

**Symptom:** pre-flight check says the IMU failed to initialize, or heading
does not respond.

**Fix:**

1. The robot still drives (encoder heading). It is a WARN, not a FAIL.
2. Check the Control Hub is seated and powered.
3. Check the IMU orientation (Chapter 5.6): logo RIGHT, USB UP. Wrong values
   do not crash the IMU — they just make heading wrong.
4. Run **Drive Direction Check**, hold **X**, rotate the robot 90° CCW by
   hand. Heading must read about +90°.

## 12.4 "The flywheel does not spin up"

**Symptom:** pre-flight check says the flywheel did not reach its RPM.

| Cause | Check |
|-------|-------|
| Motor not configured | Pre-flight check reports it missing (12.1) |
| Battery too flat | Pre-flight check battery line (12.11) |
| Wheel jammed | Turn the flywheel by hand — it should spin freely |
| Direction flag is wrong | Run **Flywheel Direction Test** (8.2) |

## 12.5 "The robot drives but the pose is wrong"

Run **Drive Direction Check** (Chapter 6.3) and read the table there.

| Reading | Cause | Fix |
|---------|-------|-----|
| x correct, y drifts | Mecanum slip, or a wheel binding | Check wheel tightness; dead wheels would fix it |
| x and y both wrong | Wrong polarity, or the localizer is confused | Chapter 6.2 |
| Pose does not change at all | Localizer not running | Check the IMU (12.3); check the dead wheel names |

## 12.6 "The robot drives BACKWARD when I push forward"

Motor polarity. Chapter 6.2. Note that flipping **all four** flags reverses
the robot; flipping one or two makes it strafe or spin.

## 12.7 "The build fails"

```bash
./gradlew :TeamCode:assembleDebug
```

The **first** `e:` line is the real error. Common ones:

| First `e:` line | Meaning | Fix |
|-----------------|---------|-----|
| `Unresolved reference 'X'` | A symbol does not exist | Check the spelling; it may live on another branch |
| `SDK location not found` | `local.properties` missing | Run the setup script (Chapter 2) |
| `Unsupported class file major version` | Java too old | JDK 17+ |
| `Could not resolve all artifacts` | Network/proxy | Check the network and retry |

Then also run the tests: `./gradlew :TeamCode:test`.

## 12.8 "The app installs but does not start"

Chapter 3.4. Get the crash:

```bash
adb logcat -d -t 400 | grep -iE "FATAL EXCEPTION|AndroidRuntime"
```

## 12.9 "Balls jam in the loader"

| Constant | Default | Adjust if |
|----------|---------|-----------|
| `LOADER_STROKE_MS` | 800 | The ball does not fully enter → raise |
| `LOADER_RETRACT_MS` | 600 | The next ball does not seat → raise |
| `LOADER_STROKE_END_POSITION` | 1.0 | The ball is pushed too far/short → tune |
| `LOADER_HOME_POSITION` | 0.0 | The gate does not close → tune |

## 12.10 "The robot cuts out its power"

**Symptom:** drive power drops to zero unexpectedly.

**Cause:** the tilt guard. If the robot is tilted more than `TILT_GUARD_DEG`
(28°), the code cuts drive power — this prevents the robot from driving off
the field.

**Fix:** put the robot flat. If it happens on flat ground, the IMU orientation
is wrong (Chapter 5.6).

## 12.11 "The battery is low / the robot is slow"

Pre-flight check prints the battery voltage. Under load it must stay above
`BATTERY_MIN_VOLTS` (12.0 V).

| Symptom | Fix |
|---------|-----|
| Voltage below 12 V | Change the battery |
| Voltage collapses when the flywheel spins | The battery is old or the connector is loose |
| Everything slow at 13 V | The flywheels draw a lot — expected |

## 12.12 "Nothing happens when I press START"

| Cause | Check |
|-------|-------|
| The OpMode is not selected | Driver Station → INIT → select it → START |
| It is waiting in INIT | The code prints a banner during INIT; that is normal |
| The robot is on the wrong mode | Check the DS mode selector is on the robot |

## 12.13 "Vision sees nothing"

| Check | How |
|-------|-----|
| The right camera is doing the job | Ball detection uses the **FRONT** camera |
| Focal length is calibrated | Chapter 9.3 |
| HSV ranges are tuned for the lighting | Chapter 9.4 |
| The camera is actually plugged in | Try a different USB port |

## 12.14 "The robot is not on the Driver Station opmode list"

The APK is old or failed to install. Re-flash (Chapter 3) and confirm
`FLASH COMPLETE AND VERIFIED`.

## 12.15 The full OpMode catalog

Every OpMode on the Driver Station, what it is for, and when to use it.

### Test group — diagnostics (use these to fix things)

| OpMode | What it does | Use it when |
|--------|--------------|-------------|
| **PRE-FLIGHT CHECK** | Checks every subsystem, prints PASS/WARN/FAIL | **Before every match.** Start here |
| **Wheel Direction Test** | Spins one drive wheel at a time | The robot drives the wrong way (6.2) |
| **Flywheel Direction Test** | Spins one flywheel at a time | The ball shoots backward or sideways (8.2) |
| **Drive Direction Check** | Drives 50 cm, shows the pose | The robot's position estimate is wrong (6.3) |
| **Drivetrain Setup & Verify** | Shows all derived constants; spin test, 50 cm crawl, live encoders | You want to see every derived number |
| **Flywheel Calibration** | Steps the flywheel through RPMs and fires one ball per step | You are measuring the shooter curve (8.4) |
| **Vision Test OpMode** | Live HSV tuning for every game piece color | The robot cannot see balls (9.4) |
| **Visual Localization Test** | 6 vision checks (stationary, rotation, translation, ...) | Before enabling autonomous vision (9.5) |
| **Camera Calibration (C270)** | Measures the camera focal length | Distances or aiming are wrong (9.3) |
| **Drive Only (hub + motors)** | Drives with ONLY the hub and drive motors | Isolating a drive problem from everything else |
| **Neural Net Test** | Shows the neural detector and P(hit) table | Diagnostic only — classical CV is what the robot uses |
| **Experiment (test bench)** | Runs a scripted bench sequence | Writing your own bench test |

### The two competition OpModes

| OpMode | Group | Use it for |
|--------|-------|------------|
| **BioBuzz TeleOp** | TeleOp | Driver-controlled match |
| **BioBuzz Autonomous** | Auto | Autonomous match |

### Diagnostic OpMode

| OpMode | Group | Use it for |
|--------|-------|------------|
| **BioBuzz SelfPlay TeleOp** | TeleOp | Watching what the AI *would* do while a human drives. Diagnostic only — it does not drive the robot |

### Auto group

| OpMode | Group | Use it for |
|--------|-------|------------|
| **Simple 50cm Out-Back** | Auto | The simplest possible autonomous check — drive 50 cm and return. **Run this before trusting the real autonomous** |
| **BioBuzz Autonomous** | Auto | The real autonomous |

## 12.16 "Everything is broken and I have no idea"

Do these in order and read every output:

```bash
bash scripts/deploy_to_hub.sh     # does it build, install, and run?
```
Then on the robot: **PRE-FLIGHT CHECK**. Then use the FAIL lines to pick the
section above.

If the build fails, fix that first — nothing else can be diagnosed until the
code runs.

---

# Chapter 13: Advanced — Extending the Codebase

## 13.1 Repository layout

```
16660-BioBuzz/
├── build.gradle / build.common.gradle / build.dependencies.gradle
├── settings.gradle                    # Includes :FtcRobotController, :TeamCode
├── local.properties                   # YOUR SDK path — gitignored, generated by setup scripts
├── scripts/                           # setup + deploy for macOS, Linux, Windows
│   ├── setup-mac.sh
│   ├── setup-linux.sh
│   ├── setup-windows.ps1
│   ├── deploy_to_hub.sh
│   └── deploy_to_hub.ps1
├── docs/LEGALITY-REVIEW.md            # Black Box rules analysis
├── tools/                             # Python desktop tools (HSV, flywheel, blackbox)
├── TeamCode/src/main/java/org/firstinspires/ftc/teamcode/
│   ├── globals.kt                     # ALL tunables except drive polarity
│   ├── driveconfig.kt                 # Drive polarity + mixing (single source of truth)
│   ├── drivetrainspec.kt              # Physical measurements → derived constants
│   ├── motors.kt                      # Drivetrain, Intake, FlyWheel, Loader, ShotDetector
│   ├── hardware.kt                    # HardwareProbe, IntakeJamGuard, TiltGuard
│   ├── autonomy.kt                    # PhaseClock, ShooterModel, WorldModel (pure Kotlin)
│   ├── hivescoring.kt                 # HiveScoring, BallLocator, TagInfoOverlay
│   ├── cameras.kt                     # Legacy VisionSystem
│   ├── ControlFrame.kt                # 20 ms motor-poll frame limiter
│   ├── odometry.kt                    # DeadWheelMath (pure Kotlin)
│   ├── debug.kt                       # Overlays
│   ├── PreFlightCheck.kt              # THE pre-match check OpMode
│   ├── DriveDirectionTests.kt         # Wheel / Flywheel / Drive direction tests
│   ├── ReenterStartingAreaAuto.kt     # "Simple 50cm Out-Back" autonomous
│   ├── BioBuzzAutonomous.kt           # MAIN autonomous
│   ├── BioBuzzTeleOp.kt               # MAIN teleop
│   ├── BioBuzzSelfPlayTeleOp.kt       # Shadow-mode planner telemetry
│   ├── TestOpModes.kt                 # Vision, camera calibration, drivetrain setup, NN
│   ├── changelog.md                   # Write a summary of your changes here
│   ├── roadrunner/MecanumDrive.kt     # RR 1.0 drive + localizers + safe IMU
│   ├── roadrunner/Localizer.kt
│   ├── roadrunner/Drawing.kt
│   └── vision/                        # VisionConfig, tracking, fusion
└── TeamCode/src/test/                 # 50 JVM unit tests (no Android needed)
```

## 13.2 Design principles

| Principle | How it shows up |
|-----------|-----------------|
| **Single source of truth** | Measurements in `drivetrainspec.kt`; polarity in `driveconfig.kt`; everything else derived |
| **Pure logic separated** | `autonomy.kt`, `odometry.kt`, `drivetrainspec.kt`, `driveconfig.kt` have no Android imports → unit-testable |
| **No singletons** | Hardware is created per OpMode and passed explicitly |
| **Nothing optional is fatal** | `HardwareProbe` + try/catch around every optional device |
| **Never ship a zero gain** | RR feedforward and gains are derived, not zero |
| **One mix matrix** | Both drive paths call `DriveConfig.mix()` |

## 13.3 Adding a new OpMode

```kotlin
@TeleOp(name = "My OpMode", group = "Test")
class MyOpMode : LinearOpMode() {
    override fun runOpMode() {
        val hw = HardwareProbe.probe(hardwareMap)
        if (!hw.driveMotors) {
            // Degrade, do not crash.
            while (!isStopRequested) {
                telemetry.addLine("No drive motors configured")
                telemetry.update()
                sleep(50)
            }
            return
        }
        val drivetrain = Drivetrain()
        waitForStart()
        while (opModeIsActive()) {
            NextMotor.motorEventLoop.poll()   // REQUIRED every loop
            drivetrain.driveTeleOp(0.0, 0.0, 0.0)
            telemetry.update()
        }
        drivetrain.flushMotors()
    }
}
```

**Critical:** call `NextMotor.motorEventLoop.poll()` every loop. Without it,
velocity and position control never update and the motors stay inert.

## 13.4 Adding a new hardware device

1. Add the config name to `globals.kt`:
   ```kotlin
   const val MY_NEW_MOTOR = "my_motor"
   ```
2. Add it to `HardwareProbe.probe()` so the pre-flight check reports it.
3. Create the mechanism class in `motors.kt`.
4. Add it to the Robot Configuration on the robot.
5. Construct it **only if it exists** — a motor for a missing device crashes the
   app on a background thread.

## 13.5 Writing tests

```bash
./gradlew :TeamCode:test
./gradlew :TeamCode:test --tests "org.firstinspires.ftc.teamcode.DriveConfigTest"
```

Tests must be pure Kotlin (no Android classes). Good models to copy:
`autonomy.kt`, `odometry.kt`, `drivetrainspec.kt`, `driveconfig.kt`.

`DriveConfigTest.kt` is the one to read first: it asserts the mixing matrix
matches the proven JJ scheme exactly, so a future edit that breaks the wiring
fails the build instead of failing at the competition.

## 13.6 Making a change safely

1. Note what you are changing and why.
2. Change ONE thing.
3. `./gradlew :TeamCode:assembleDebug :TeamCode:test`
4. `bash scripts/deploy_to_hub.sh`
5. Run **PRE-FLIGHT CHECK**.
6. Run **Wheel Direction Test** if you touched `driveconfig.kt`.
7. Write a summary in `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/changelog.md`.

---

# Chapter 14: Appendix — All Constants

## 14.1 Hardware names (`globals.kt`)

```kotlin
DrivetrainFrontLeft   = "frontleft"
DrivetrainFrontRight  = "frontright"
DrivetrainBackLeft    = "backleft"
DrivetrainBackRight   = "backright"
FlywheelLeftMotor     = "flywheelleft"
FlywheelRightMotor    = "flywheelright"
IntakeMotor           = "intake"
LoaderServoName       = "loader"
RR_IMU_NAME           = "imu"
UP_CAM_NAME           = "Webcam High"
GROUND_CAM_NAME       = "Webcam Ground"
DEAD_WHEEL_PAR0_NAME  = "backleftodo"
DEAD_WHEEL_PAR1_NAME  = "backrightodo"
DEAD_WHEEL_PERP_NAME  = "frontodo"
```

## 14.2 Drive polarity (`driveconfig.kt`)

```kotlin
FL_REVERSED = false
FR_REVERSED = true
BL_REVERSED = false
BR_REVERSED = true
```

Flywheel polarity (`globals.kt`):

```kotlin
FLYWHEEL_LEFT_REVERSED  = false
FLYWHEEL_RIGHT_REVERSED = true
```

## 14.3 Physical measurements (`drivetrainspec.kt`)

```kotlin
DRIVE_WHEEL_DIAMETER_CM = 10.4
DRIVE_MOTOR_CPR         = 28.0
DRIVE_GEAR_RATIO        = 19.2
DRIVE_MOTOR_MAX_RPM     = 312.0
TRACK_WIDTH_CM          = 42.0
WHEELBASE_CM            = 32.0
MECANUM_LATERAL_SLIP    = 1.0

DEAD_WHEEL_DIAMETER_CM     = 3.8
DEAD_WHEEL_TICKS_PER_REV   = 4096.0
PAR0_X_CM = -16.0; PAR0_Y_CM =  8.5; PAR0_REVERSED = false
PAR1_X_CM = -16.0; PAR1_Y_CM = -8.5; PAR1_REVERSED = false
PERP_X_CM =  -1.0; PERP_Y_CM = 21.0; PERP_REVERSED = false
```

## 14.4 Motor specs and gains (`globals.kt`)

```kotlin
DRIVE_MOTOR_MAX_RPM      = 312.0     // from DrivetrainSpec
FLYWHEEL_MOTOR_MAX_RPM   = 6000.0
FLYWHEEL_MOTOR_ENCODER_CPR = 28.0
FLYWHEEL_PRIME_RPM       = 2000.0
FLYWHEEL_SPEED_TOLERANCE_RPM = 150.0
FLYWHEEL_FEED_TIME_MS    = 1200.0
FLYWHEEL_MIN_FIRING_RPM  = 1000.0
INTAKE_MOTOR_MAX_RPM     = 2500.0
INTAKE_MOTOR_ENCODER_CPR = 28.0
```

## 14.5 Road Runner defaults (`roadrunner/MecanumDrive.kt`)

```kotlin
logoFacingDirection = RIGHT
usbFacingDirection  = UP

inPerTick       = DrivetrainSpec.rrInPerTick        // derived
lateralInPerTick = DrivetrainSpec.rrLateralInPerTick  // derived
trackWidthTicks = DrivetrainSpec.rrTrackWidthTicks   // derived

kS = DrivetrainSpec.RR_KS   // 0.6 volts
kV = DrivetrainSpec.rrKV    // derived from free speed
kA = DrivetrainSpec.RR_KA   // 0.0

maxWheelVel    = 50.0
minProfileAccel = -30.0
maxProfileAccel = 50.0
maxAngVel      = Math.PI
maxAngAccel    = Math.PI

axialGain   = 2.0
lateralGain = 2.0
headingGain = 2.0
axialVelGain   = 0.5
lateralVelGain = 0.5
headingVelGain = 0.5
```

## 14.6 Shooter (`globals.kt`)

```kotlin
SHOOTER_CALIBRATION_RPM = doubleArrayOf(6000.0, 5000.0, 4000.0, 3000.0, 2000.0, 1000.0)
SHOOTER_CALIBRATION_DISTANCE_CM = doubleArrayOf(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)  // fill in!
SHOOTER_FLYWHEEL_RADIUS_CM = 3.8
SHOOTER_TRANSFER_EFFICIENCY = 0.55
SHOT_DIP_THRESHOLD_RPM = 250.0
SHOT_DETECT_WINDOW_MS = 900.0
SHOT_RECOVERY_TIMEOUT_MS = 2500.0

LOADER_HOME_POSITION = 0.0
LOADER_STROKE_END_POSITION = 1.0
LOADER_STROKE_MS = 800.0
LOADER_RETRACT_MS = 600.0
```

## 14.7 Shooting window (`hivescoring.kt`)

```kotlin
SHOOT_MIN_RANGE_CM   = 60.0
SHOOT_MAX_RANGE_CM   = 140.0   // fallback until the curve is measured
SHOOT_MAX_BEARING_DEG = 10.0
RANGE_CENTER_CM = (SHOOT_MIN_RANGE_CM + SHOOT_MAX_RANGE_CM) / 2.0
RANGE_GAIN = 0.004
BEARING_GAIN = 0.02
ASSIST_MAX_POWER = 0.30
CAPTURE_APPROACH_CM = 18.0
CAPTURE_RANGE_GAIN = 0.02
CAPTURE_BEARING_GAIN = 0.02
ALIGN_TIMEOUT_MS = 8000.0
```

## 14.8 Autonomous timing (`globals.kt`)

```kotlin
AUTONOMY_PERIOD_SEC = 30.0
AUTONOMY_PARK_RESERVE_SEC = 3.0
AUTONOMY_FIRE_RESERVE_SEC = 6.0
MIN_CYCLE_REMAINING_SEC = 14.0
AUTONOMOUS_COLLECT_TIMEOUT_MS = 9000.0
FLOWER_PULL_TIMEOUT_MS = 6000.0
FLOWER_APPROACH_CM = 25.0
FLOWER_SETTLE_AFTER_PULL_MS = 600L
MAX_BALLS_HELD = 4
```

## 14.9 Collection, dodge, recovery (`globals.kt`)

```kotlin
CAPTURE_TURN_GAIN = 0.008
CAPTURE_TURN_MAX_POWER = 0.20
TARGET_SWITCH_CONFIRM_LOOPS = 3
AVOID_ROBOT_BEARING_DEG = 15.0
PICKUP_CURRENT_BUMP_A = 1.5
PICKUP_BUMP_MIN_MS = 60.0
PICKUP_BUMP_MAX_MS = 450.0
PICKUP_CONFIRM_CM = 25.0
PICKUP_SETTLE_MS = 400.0
SEARCH_SWEEP_TURN_POWER = 0.12
BLOCKED_GIVEUP_MS = 1500.0
INTAKE_JAM_CURRENT_A = 3.5
INTAKE_JAM_SUSTAIN_MS = 300.0
INTAKE_COUGH_MS = 450.0
INTAKE_MAX_COUGHS_PER_JAM = 3
TILT_GUARD_DEG = 28.0
```

## 14.10 World model (`globals.kt`)

```kotlin
WM_TRACK_MATCH_CM = 40.0
WM_TRACK_EMA_ALPHA = 0.4
WM_VISIBLE_FRESH_SEC = 0.3
DEAD_RECKON_MAX_AGE_SEC = 3.0
CHASE_MIN_CONTOUR_AREA_PX = 350.0
WM_CONFIDENCE_TAU_SEC = 10.0
WM_BALL_MAX_AGE_SEC = 45.0
WM_ROBOT_MAX_AGE_SEC = 6.0
LAST_SLOT_PREFER_HIGHEST_VALUE = true
```

## 14.11 Teleop (`globals.kt`)

```kotlin
TELEOP_FIELD_CENTRIC_DEFAULT = false
TELEOP_POSE_HOLD_DEFAULT = true
TELEOP_INPUT_DEADZONE = 0.08
TELEOP_HOLD_POSITION_KP = 0.06
TELEOP_HOLD_HEADING_KP = 0.02
TELEOP_HOLD_MAX_POWER = 0.25
```

## 14.12 Vision (`vision/VisionConfig.kt`)

```kotlin
C270_FOCAL_LENGTH_PIXELS = 543.0   // MEASURE THIS
CAMERA_WIDTH_PIXELS = 800
CAMERA_HEIGHT_PIXELS = 448
GROUND_CAM_WIDTH = 640
GROUND_CAM_HEIGHT = 480
POLLEN_PHYSICAL_WIDTH_MM = 71.12
NECTAR_PHYSICAL_WIDTH_MM = 91.44
APRILTAG_PHYSICAL_WIDTH_MM = 82.55
FLOWER_PHYSICAL_WIDTH_MM = 101.5
ROBOT_PHYSICAL_WIDTH_MM = 450.0
```

`VisionTuning` — see the table in Chapter 9.6.

## 14.13 Quick reference card

| Task | OpMode | File to edit |
|------|--------|--------------|
| Verify motor directions | Wheel Direction Test | `driveconfig.kt` |
| Verify odometry | Drive Direction Check | `driveconfig.kt` |
| Verify flywheel direction | Flywheel Direction Test | `globals.kt` |
| Prove the robot is ready | **PRE-FLIGHT CHECK** | — |
| Tune HSV ranges | Vision Test OpMode | `globals.kt` |
| Calibrate camera focal length | Camera Calibration (C270) | `vision/VisionConfig.kt` |
| Calibrate the shooter | Flywheel Calibration | `globals.kt` |
| Tune RR drive | (FTC Dashboard) | `roadrunner/MecanumDrive.kt` |
| Simplest autonomous check | Simple 50cm Out-Back | — |
| Run the match | BioBuzz TeleOp / BioBuzz Autonomous | — |
| Flash the robot | — | `scripts/deploy_to_hub.sh` |

---

# Chapter 15: Glossary

| Term | Meaning |
|------|---------|
| **ADB** | The Android Debug Bridge — the command-line tool that talks to the Control Hub over USB |
| **APK** | The Android app package the robot runs. Built by Gradle, installed with adb |
| **Control Hub / Driver Hub** | The REV hub that runs the robot's code |
| **Dead wheels** | Unpowered wheels that only have encoders; they measure travel without slip |
| **Encoder** | A sensor that counts motor rotation |
| **Feedforward** | An open-loop estimate of the motor power needed for a given speed |
| **FL / FR / BL / BR** | Front-left, front-right, back-left, back-right wheels |
| **FPS** | Frames per second |
| **FSM** | Finite state machine |
| **gRPC / @Config** | The FTC Dashboard mechanism for live-tunable settings |
| **HSV** | Hue/saturation/value — the color space the ball detector uses |
| **IMU** | Inertial Measurement Unit — the hub's internal compass/gyro |
| **kV / kS** | Road Runner's velocity and static-friction feedforward, in volts |
| **Localizer** | The class that turns encoder readings into a robot position |
| **Mecanum** | The wheel arrangement that lets the robot strafe |
| **Odometry** | Estimating the robot's position from wheel encoders |
| **OpMode** | A robot program the Driver Station can start |
| **Pinpoint / pose** | Where the robot thinks it is on the field |
| **Road Runner (RR)** | The trajectory-following and localization library |
| **RPM** | Revolutions per minute |
| **TeleOp** | Driver-controlled mode |
| **Autonomous** | Robot-controlled mode |
| **WARN / FAIL** | Pre-flight check verdicts: missing-but-survivable / broken |
| **X configuration** | The mecanum wheel layout where the wheels sit in an X |

---

*BioBuzz 2026–27 season. This manual covers the `main` branch.*
