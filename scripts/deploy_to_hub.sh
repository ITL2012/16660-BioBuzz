#!/usr/bin/env bash
# =============================================================================
# deploy_to_hub.sh — build, flash, and verify the robot
#
#   bash scripts/deploy_to_hub.sh            # one shot
#   bash scripts/deploy_to_hub.sh --wait     # wait for the Control Hub to appear
#
# This is the ONLY command you need to put the code on the robot. It:
#   1. builds the APK
#   2. waits for the Control Hub on USB (unless --wait is off)
#   3. installs the APK
#   4. launches the Robot Controller app
#   5. VERIFIES the app is running and did not crash
#
# "Success" from adb is NOT proof of a working flash. Step 5 is the proof.
# =============================================================================
set -uo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
cd "$PROJECT_DIR"

RED='\033[0;31m'; GREEN='\033[0;32m'; YELLOW='\033[1;33m'; BLUE='\033[0;34m'; NC='\033[0m'
say()  { echo -e "${BLUE}==>${NC} $*"; }
ok()   { echo -e "${GREEN}  OK${NC} $*"; }
warn() { echo -e "${YELLOW}  WARN${NC} $*"; }
die()  { echo -e "${RED}  ERROR${NC} $*" >&2; exit 1; }

WAIT_FOR_DEVICE=0
[ "${1:-}" = "--wait" ] && WAIT_FOR_DEVICE=1

echo "======================================================================"
echo " Flash the robot"
echo "======================================================================"

# ---------------------------------------------------------- 1. locate adb
say "Locating adb..."
ADB=""
if [ -f local.properties ]; then
    SDK_DIR="$(sed -n 's/^sdk\.dir=//p' local.properties | head -1)"
    SDK_DIR="${SDK_DIR//\\:/:}"
    if [ -x "$SDK_DIR/platform-tools/adb" ]; then
        ADB="$SDK_DIR/platform-tools/adb"
    fi
fi
if [ -z "$ADB" ] && command -v adb >/dev/null 2>&1; then
    ADB="$(command -v adb)"
fi
if [ -z "$ADB" ]; then
    die "adb not found. Run the setup script for your OS first (scripts/setup-mac.sh etc.)."
fi
ok "adb: $ADB"

# ------------------------------------------------------------- 2. build
say "Building the APK..."
./gradlew :TeamCode:assembleDebug --console=plain || die "Build failed (see errors above)."
APK="TeamCode/build/outputs/apk/debug/TeamCode-debug.apk"
[ -f "$APK" ] || die "Build reported success but $APK is missing."
ok "APK: $APK"

# -------------------------------------------------------- 3. find device
say "Looking for the Control Hub..."
find_device() {
    "$ADB" devices | grep -qE "^[0-9a-f]+[[:space:]]+device"
}

if find_device; then
    ok "device connected"
elif [ "$WAIT_FOR_DEVICE" -eq 1 ]; then
    echo "    Waiting for the Control Hub to appear on USB..."
    echo "    (plug in a USB-C DATA cable — a charge-only cable will NOT work)"
    for i in $(seq 1 120); do
        if find_device; then
            ok "device connected"
            break
        fi
        sleep 5
    done
    find_device || die "No Control Hub appeared after 10 minutes. Check the cable and USB port."
else
    die "No Control Hub found on USB. Plug it in with a USB-C DATA cable, or run with --wait to wait for it."
fi

"$ADB" devices -l | sed 's/^/    /'

# ---------------------------------------------------------- 4. install
say "Installing..."
"$ADB" install -r "$APK" || die "Install failed. Try: $ADB uninstall com.qualcomm.ftcrobotcontroller, then re-run."
ok "installed"

# ---------------------------------------------------------- 5. launch
say "Launching the Robot Controller app..."
"$ADB" shell am force-stop com.qualcomm.ftcrobotcontroller >/dev/null 2>&1 || true
sleep 2
"$ADB" shell am start -n \
    com.qualcomm.ftcrobotcontroller/org.firstinspires.ftc.robotcontroller.internal.FtcRobotControllerActivity \
    >/dev/null 2>&1 || warn "Could not auto-launch; start the app on the robot by hand."
sleep 10

# ---------------------------------------------------------- 6. VERIFY
say "Verifying the app actually started..."
PID="$("$ADB" shell pidof com.qualcomm.ftcrobotcontroller 2>/dev/null | tr -d '\r')"
if [ -z "$PID" ]; then
    CRASH="$("$ADB" logcat -d -t 400 2>/dev/null | grep -iE 'FATAL EXCEPTION|AndroidRuntime' | tail -10)"
    echo ""
    warn "The app is NOT running. Something crashed on launch."
    [ -n "$CRASH" ] && { echo "$CRASH"; echo ""; }
    echo "  Common causes:"
    echo "    - A hardware device in the Robot Configuration is missing or misnamed."
    echo "    - The APK is older than the code (rebuild: ./gradlew :TeamCode:assembleDebug)"
    echo "  The robot will still work with the PREVIOUS app if one was installed."
    exit 1
fi
ok "app running (pid $PID)"

CRASHES="$("$ADB" logcat -d -t 400 2>/dev/null | grep -icE 'FATAL EXCEPTION' || true)"
if [ "${CRASHES:-0}" -gt 0 ]; then
    warn "Found $CRASHES crash entries in logcat — check with:"
    warn "  $ADB logcat -d -t 400 | grep -iE 'FATAL EXCEPTION|AndroidRuntime'"
fi

# Show the saved robot configs so the person knows what is configured.
say "Saved robot configurations on the hub:"
"$ADB" shell 'ls /sdcard/FIRST/*.xml' 2>/dev/null | tr -d '\r' | sed 's/^/    /' || true

echo ""
echo "======================================================================"
echo -e "${GREEN} FLASH COMPLETE AND VERIFIED${NC}"
echo "======================================================================"
echo "On the robot:"
echo "  1. Open the Driver Station, pick 'PRE-FLIGHT CHECK' in the Test group."
echo "  2. Press START. Read the report: any FAIL means do not start a match."
echo "  3. Then pick 'BioBuzz TeleOp' to drive."
echo ""
