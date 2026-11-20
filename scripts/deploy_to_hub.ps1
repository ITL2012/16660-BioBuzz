# =============================================================================
# deploy_to_hub.ps1 - build, flash, and verify the robot (Windows)
#
#   powershell -ExecutionPolicy Bypass -File scripts\deploy_to_hub.ps1
#   powershell -ExecutionPolicy Bypass -File scripts\deploy_to_hub.ps1 -Wait
#
# "Success" from adb is NOT proof of a working flash. The last step verifies.
# =============================================================================

param([switch]$Wait)

function Say  { Write-Host "==> $args" -ForegroundColor Cyan }
function Ok   { Write-Host "  OK   $args" -ForegroundColor Green }
function Warn { Write-Host "  WARN $args" -ForegroundColor Yellow }
function Die  { Write-Host "  ERROR $args" -ForegroundColor Red; exit 1 }

$ProjectDir = Split-Path -Parent (Split-Path -Parent $MyInvocation.MyCommand.Path)
Set-Location $ProjectDir

Write-Host "======================================================================"
Write-Host " Flash the robot (Windows)"
Write-Host "======================================================================"

# ---------------------------------------------------------- 1. locate adb
Say "Locating adb..."
$Adb = $null
if (Test-Path "local.properties") {
    $line = Select-String -Path "local.properties" -Pattern '^sdk\.dir=(.*)$' | Select-Object -First 1
    if ($line) {
        $sdk = ($line.Matches[0].Groups[1].Value -replace '\\:', ':') -replace '\\', '/'
        $candidate = Join-Path $sdk "platform-tools/adb.exe"
        if (Test-Path $candidate) { $Adb = $candidate }
    }
}
if (-not $Adb) {
    $cmd = Get-Command adb -ErrorAction SilentlyContinue
    if ($cmd) { $Adb = $cmd.Source }
}
if (-not $Adb) { Die "adb not found. Run scripts\setup-windows.ps1 first." }
Ok "adb: $Adb"

# ------------------------------------------------------------- 2. build
Say "Building the APK..."
& "$ProjectDir\gradlew.bat" :TeamCode:assembleDebug --console=plain
if ($LASTEXITCODE -ne 0) { Die "Build failed (see errors above)." }
$Apk = Join-Path $ProjectDir "TeamCode\build\outputs\apk\debug\TeamCode-debug.apk"
if (-not (Test-Path $Apk)) { Die "Build succeeded but the APK is missing." }
Ok "APK: $Apk"

# -------------------------------------------------------- 3. find device
Say "Looking for the Control Hub..."
function Find-Device {
    $out = & $Adb devices 2>$null
    return ($out | Select-String -Pattern '^[0-9a-f]+\s+device$') -ne $null
}

if (Find-Device) {
    Ok "device connected"
} elseif ($Wait) {
    Write-Host "    Waiting for the Control Hub on USB..."
    Write-Host "    (plug in a USB-C DATA cable - a charge-only cable will NOT work)"
    for ($i = 0; $i -lt 120; $i++) {
        if (Find-Device) { Ok "device connected"; break }
        Start-Sleep -Seconds 5
    }
    if (-not (Find-Device)) { Die "No Control Hub appeared after 10 minutes. Check the cable." }
} else {
    Die "No Control Hub found on USB. Plug it in with a USB-C DATA cable, or re-run with -Wait."
}
& $Adb devices -l | ForEach-Object { Write-Host "    $_" }

# ---------------------------------------------------------- 4. install
Say "Installing..."
& $Adb install -r $Apk
if ($LASTEXITCODE -ne 0) {
    Die "Install failed. Try: & $Adb uninstall com.qualcomm.ftcrobotcontroller, then re-run."
}
Ok "installed"

# ---------------------------------------------------------- 5. launch
Say "Launching the Robot Controller app..."
& $Adb shell am force-stop com.qualcomm.ftcrobotcontroller 2>$null | Out-Null
Start-Sleep -Seconds 2
& $Adb shell am start -n com.qualcomm.ftcrobotcontroller/org.firstinspires.ftc.robotcontroller.internal.FtcRobotControllerActivity 2>$null | Out-Null
Start-Sleep -Seconds 10

# ---------------------------------------------------------- 6. VERIFY
Say "Verifying the app actually started..."
$Pid = (& $Adb shell pidof com.qualcomm.ftcrobotcontroller 2>$null | Out-String).Trim()
if (-not $Pid) {
    Write-Host ""
    Warn "The app is NOT running - something crashed on launch."
    & $Adb logcat -d -t 400 2>$null | Select-String -Pattern "FATAL EXCEPTION|AndroidRuntime" | Select-Object -Last 10 | ForEach-Object { Write-Host "    $_" }
    Write-Host ""
    Write-Host "  Common causes:"
    Write-Host "    - A hardware device in the Robot Configuration is missing or misnamed."
    Write-Host "    - The APK is older than the code (rebuild)."
    Write-Host "  The robot will still work with the PREVIOUS app if one was installed."
    exit 1
}
Ok "app running (pid $Pid)"

Say "Saved robot configurations on the hub:"
& $Adb shell 'ls /sdcard/FIRST/*.xml' 2>$null | ForEach-Object { Write-Host "    $_" }

Write-Host ""
Write-Host "======================================================================"
Write-Host " FLASH COMPLETE AND VERIFIED"
Write-Host "======================================================================"
Write-Host "On the robot:"
Write-Host "  1. Open the Driver Station, pick 'PRE-FLIGHT CHECK' in the Test group."
Write-Host "  2. Press START. Read the report: any FAIL means do not start a match."
Write-Host "  3. Then pick 'BioBuzz TeleOp' to drive."
Write-Host ""
