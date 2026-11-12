package org.firstinspires.ftc.teamcode

import android.util.Size
import com.qualcomm.robotcore.hardware.Gamepad
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotorEx
import dev.nextftc.hardware.actuators.NextMotor
import org.firstinspires.ftc.teamcode.ControlFrame
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName
import org.firstinspires.ftc.teamcode.vision.CameraId
import org.firstinspires.ftc.teamcode.vision.VisionConfig
import org.firstinspires.ftc.teamcode.vision.VisionSubsystem
import org.firstinspires.ftc.teamcode.vision.VisionTuning
import org.firstinspires.ftc.vision.VisionPortal
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor
import org.firstinspires.ftc.vision.apriltag.AprilTagSingleDetection
import org.opencv.core.Point
import org.opencv.core.Scalar
import java.util.Locale
import kotlin.math.atan
import kotlin.math.sqrt

/**
 * ============ TEST / DEBUG OPMODES (all in one place) ============
 *
 *  - VisionTestOpMode        — live HSV tuning GUI for ball/flower colors
 *  - VisualLocalizationTest  — feature-tracking/localization field tests 1-6
 *  - CameraCalibrationOpMode — C270 focal-length calibration GUI
 *
 * All register as group "Test" on the Driver Station. Debug overlay
 * processors (MetadataOverlay, CalibrationProcessor, CalibrationOverlay)
 * live in debug.kt.
 */
@TeleOp(name = "Vision Test OpMode", group = "Test")
class VisionTestOpMode : LinearOpMode() {

    private enum class Target { POLLEN, NECTAR_BLUE, NECTAR_RED, NECTAR_RED_WRAP, FLOWER }
    private var currentTarget = Target.POLLEN
    
    private var hMin = 0.0
    private var hMax = 180.0
    private var sMin = 100.0
    private var sMax = 255.0
    private var vMin = 100.0
    private var vMax = 255.0

    private var selectedParam = 0 // 0: Hmin, 1: Hmax, 2: Smin, 3: Smax, 4: Vmin, 5: Vmax
    private var calibrationEnabled = false
    private var prevA = false // edge-detect the A toggle (hold = one toggle, not a machine gun)

    override fun runOpMode() {
        val visionSystem = VisionSystem(hardwareMap)

        telemetry.addData("Status", "Initialized")
        telemetry.addLine("A: Toggle Calibration Mode")
        telemetry.addLine("DPAD Up/Down: Select Parameter")
        telemetry.addLine("DPAD Left/Right: Tweak Value (X for 10x)")
        telemetry.addLine("Bumpers: Switch Target Piece")
        telemetry.update()

        // Sync initial values
        loadTargetDefaults(currentTarget)

        waitForStart()

        while (opModeIsActive()) {
            // Target Selection
            if (gamepad1.left_bumper) {
                val values = Target.entries
                currentTarget = values[(currentTarget.ordinal - 1 + values.size) % values.size]
                loadTargetDefaults(currentTarget)
                sleep(200)
            } else if (gamepad1.right_bumper) {
                val values = Target.entries
                currentTarget = values[(currentTarget.ordinal + 1) % values.size]
                loadTargetDefaults(currentTarget)
                sleep(200)
            }

            // Parameter Selection
            if (gamepad1.dpad_up) {
                selectedParam = (selectedParam - 1 + 6) % 6
                sleep(200)
            } else if (gamepad1.dpad_down) {
                selectedParam = (selectedParam + 1) % 6
                sleep(200)
            }

            // Tweak Value
            val delta = if (gamepad1.x) 10.0 else 1.0
            if (gamepad1.dpad_left) {
                updateParam(-delta)
            } else if (gamepad1.dpad_right) {
                updateParam(delta)
            }

            // Calibration Mode Toggle (edge-detected: holding A flips once)
            if (gamepad1.a && !prevA) {
                calibrationEnabled = !calibrationEnabled
                visionSystem.groundPortal.setProcessorEnabled(visionSystem.calibrationProcessor, calibrationEnabled)

                // Disable production locators when calibrating
                visionSystem.groundPortal.setProcessorEnabled(visionSystem.pollenLocator, !calibrationEnabled)
                visionSystem.groundPortal.setProcessorEnabled(visionSystem.nectarBlueLocator, !calibrationEnabled)
                visionSystem.groundPortal.setProcessorEnabled(visionSystem.nectarRedLocator, !calibrationEnabled)
                visionSystem.groundPortal.setProcessorEnabled(visionSystem.nectarRedWrapLocator, !calibrationEnabled)
                visionSystem.groundPortal.setProcessorEnabled(visionSystem.flowerLocator, !calibrationEnabled)
            }
            prevA = gamepad1.a

            // Update Calibration Processor
            if (calibrationEnabled) {
                visionSystem.calibrationProcessor.min = Scalar(hMin, sMin, vMin)
                visionSystem.calibrationProcessor.max = Scalar(hMax, sMax, vMax)
            }

            val detections = visionSystem.getAllDetections()
            
            telemetry.addData("Mode", if (calibrationEnabled) "CALIBRATION (CYAN BOXES)" else "DETECTION")
            telemetry.addData("Targeting", currentTarget.name)
            telemetry.addLine("--- Tweak Values ---")
            telemetry.addLine("${if (selectedParam == 0) "> " else "  "}H Min: ${hMin.toInt()}")
            telemetry.addLine("${if (selectedParam == 1) "> " else "  "}H Max: ${hMax.toInt()}")
            telemetry.addLine("${if (selectedParam == 2) "> " else "  "}S Min: ${sMin.toInt()}")
            telemetry.addLine("${if (selectedParam == 3) "> " else "  "}S Max: ${sMax.toInt()}")
            telemetry.addLine("${if (selectedParam == 4) "> " else "  "}V Min: ${vMin.toInt()}")
            telemetry.addLine("${if (selectedParam == 5) "> " else "  "}V Max: ${vMax.toInt()}")
            telemetry.addLine("-----------------------------")

            for (detection in detections) {
                telemetry.addLine("Item: ${detection.label}")
                telemetry.addLine(String.format(Locale.US, "  Distance: %.1f mm", detection.distanceMm))
                telemetry.addLine(String.format(Locale.US, "  Angle: %.1f deg", detection.angleDegrees))
            }

            telemetry.update()
            sleep(50)
        }

        visionSystem.close()
    }

    private fun loadTargetDefaults(target: Target) {
        // Since ColorRange.min/max are protected, we use the known defaults from globals.kt
        when(target) {
            Target.POLLEN -> {
                hMin = 14.0; hMax = 35.0; sMin = 100.0; sMax = 255.0; vMin = 100.0; vMax = 255.0
            }
            Target.NECTAR_BLUE -> {
                hMin = 100.0; hMax = 140.0; sMin = 100.0; sMax = 255.0; vMin = 100.0; vMax = 255.0
            }
            Target.NECTAR_RED -> {
                hMin = 0.0; hMax = 12.0; sMin = 100.0; sMax = 255.0; vMin = 100.0; vMax = 255.0
            }
            Target.NECTAR_RED_WRAP -> {
                hMin = 168.0; hMax = 180.0; sMin = 100.0; sMax = 255.0; vMin = 100.0; vMax = 255.0
            }
            Target.FLOWER -> {
                hMin = 45.0; hMax = 80.0; sMin = 50.0; sMax = 255.0; vMin = 50.0; vMax = 255.0
            }
        }
    }

    private fun updateParam(delta: Double) {
        when(selectedParam) {
            0 -> hMin = (hMin + delta).coerceIn(0.0, 180.0)
            1 -> hMax = (hMax + delta).coerceIn(0.0, 180.0)
            2 -> sMin = (sMin + delta).coerceIn(0.0, 255.0)
            3 -> sMax = (sMax + delta).coerceIn(0.0, 255.0)
            4 -> vMin = (vMin + delta).coerceIn(0.0, 255.0)
            5 -> vMax = (vMax + delta).coerceIn(0.0, 255.0)
        }
    }
}

/**
 * ============ VISION-ONLY TEST (does NOT command the drivetrain) ============
 *
 * Field test procedure (run in order; all corrections remain disabled — this
 * OpMode constructs no drivetrain):
 *
 *  TEST 1 - Stationary robot:  features detected (blue dots turn green),
 *            stable features stay green, confidence rises, visual dx/dy/dh ~ 0.
 *  TEST 2 - Slow rotation:     static features sweep coherently; RANSAC finds
 *            the dominant rotation; outliers (RED) get rejected; dHeading
 *            matches the rotation direction/sign — else fix headingSign.
 *  TEST 3 - Translation:        push the robot forward/sideways; visual dx/dy
 *            should read +x forward (calibrate imageToRobotYawDeg until it does).
 *  TEST 4 - Dynamic obstruction: walk through the view; RED rejections dominate
 *            on the person, GREEN static features remain, confidence holds.
 *  TEST 5 - One camera blocked:  blocked cam goes stale (drops from active
 *            list), the other continues, overall confidence decreases but
 *            tracking continues.
 *  TEST 6 - Both cameras blocked: state -> LOST, confidence -> ~0, no visual
 *            motion; Road Runner would continue normally (this OpMode only
 *            proves vision output goes silent).
 *
 * Only after ALL tests pass: enable corrections in VisionAutonomous.
 */
@TeleOp(name = "Visual Localization Test", group = "Test")
class VisualLocalizationTest : LinearOpMode() {

    override fun runOpMode() {
        val vision = VisionSubsystem(hardwareMap)

        telemetry.addLine("Vision test initialized. Watch the camera previews:")
        telemetry.addLine("GREEN = static inlier, RED = outlier/moving,")
        telemetry.addLine("BLUE = new feature, YELLOW = persistent landmark.")
        telemetry.update()

        waitForStart()

        while (opModeIsActive()) {
            vision.updateRoadRunnerPose(com.acmerobotics.roadrunner.Pose2d(0.0, 0.0, 0.0))
            vision.addTelemetry(telemetry)

            val s = vision.snapshot()
            val snapshotAgeMs = if (s != null) (System.nanoTime() - s.timestampNanos) / 1e6 else 0.0
            telemetry.addData("Snapshot age ms", "%.1f".format(snapshotAgeMs))
            telemetry.update()
            sleep(50)
        }

        vision.close()
    }
}

/**
 * ============ C270 CAMERA CALIBRATION (debug GUI) ============
 *
 * Reads the real focal-length numbers for the two Logitech C270s (HIGH/UP cam
 * + GROUND/FRONT cam) straight off the Driver Station — no Android Studio, no
 * guessing. The current values in VisionConfig.kt are marked "MEASURE"; this
 * OpMode is how you measure them.
 *
 * HOW TO USE (5 minutes, one tape measure, one printed AprilTag):
 *  1. Print ANY AprilTag from the 36h11 family that is NOT a BioBuzz hive tag
 *     (IDs 30-45 are hive tags; e.g. use ID 1). Measure the width of the
 *     BLACK SQUARE in mm, edge to edge.
 *  2. Tape it flat to a wall, dead-center in front of one camera, facing the
 *     camera squarely (no tilt).
 *  3. Tape-measure the lens-to-tag distance.
 *  4. Run this OpMode. Both cameras stream stacked; the tag in view gets
 *     outlined, and telemetry shows a LIVE focal estimate.
 *  5. Match your measurements:
 *        DPAD UP/DOWN    = actual distance  (+1 cm; hold X for 10 cm)
 *        DPAD LEFT/RIGHT = printed tag width (+1 mm; hold X for 10 mm)
 *  6. Press A to HOLD — the estimate averages over frames for a stable
 *     number. Press A again to release and re-aim.
 *  7. Write the HELD AVERAGE into VisionConfig.kt:
 *        UP (high) cam  -> UP_CAMERA.intrinsics.focalLengthPx
 *        FRONT (ground) -> FRONT_CAMERA.intrinsics.focalLengthPx
 *     (and C270_FOCAL_LENGTH_PIXELS in globals.kt if you calibrate at 640x480)
 *  8. Sanity checks built into telemetry:
 *     - "dist with configured focal" should read ~ your tape-measure distance
 *     - horizontal FOV should land around 55-60 deg for a C270
 */
@TeleOp(name = "Camera Calibration (C270)", group = "Test")
class CameraCalibrationOpMode : LinearOpMode() {

    private var knownDistanceCm = 100.0
    private var tagWidthMm = APRILTAG_PHYSICAL_WIDTH_MM
    private var holding = false
    private val held = mutableListOf<Double>()
    private var prevA = false

    /** One single-tag measurement (all pixel values at the camera's stream resolution). */
    private data class TagMeasure(
        val id: Int,
        val centerPx: Point,
        /** Axis-aligned corner-quad width, px. */
        val widthPx: Double,
        /** Axis-aligned corner-quad height, px. */
        val heightPx: Double,
        /** Focal length implied by the known distance + printed tag size: f = wPx * dist / size. */
        val focalPx: Double,
    )

    override fun runOpMode() {
        val upCam = hardwareMap.get(WebcamName::class.java, HIGH_CAM_NAME)
        val frontCam = hardwareMap.get(WebcamName::class.java, GROUND_CAM_NAME)

        val upTag = tagProcessor()
        val frontTag = tagProcessor()
        val upOverlay = CalibrationOverlay()
        val frontOverlay = CalibrationOverlay()

        // Stacked live previews for both cameras on the Driver Station.
        val viewIds = VisionPortal.makeMultiPortalView(2, VisionPortal.MultiPortalLayout.VERTICAL)
        val upPortal = portal(upCam, VisionTuning.UP_CAM_WIDTH, VisionTuning.UP_CAM_HEIGHT, viewIds[0], upTag, upOverlay)
        val frontPortal = portal(frontCam, VisionTuning.FRONT_CAM_WIDTH, VisionTuning.FRONT_CAM_HEIGHT, viewIds[1], frontTag, frontOverlay)

        telemetry.addLine("C270 calibration: show ONE printed (non-hive) AprilTag")
        telemetry.addLine("squarely at one camera; enter distance & tag size.")
        telemetry.addData("Default distance", "%.0f cm", knownDistanceCm)
        telemetry.addData("Default tag width", "%.1f mm (3.25in BioBuzz tag)", tagWidthMm)
        telemetry.addLine("DPAD UP/DOWN: distance | DPAD L/R: tag width | X: 10x")
        telemetry.addLine("A: hold/release averaging")
        telemetry.update()

        waitForStart()

        while (opModeIsActive()) {
            handleControls()

            val up = measure(upTag)
            val front = measure(frontTag)
            val upCfg = VisionConfig.calibrationFor(CameraId.UP).intrinsics
            val frontCfg = VisionConfig.calibrationFor(CameraId.FRONT).intrinsics

            upOverlay.text = overlayText("UP (high cam)", up, upCfg.focalLengthPx)
            frontOverlay.text = overlayText("FRONT (ground cam)", front, frontCfg.focalLengthPx)

            if (holding) {
                up?.let { held.add(it.focalPx) }
                front?.let { held.add(it.focalPx) }
            }

            telemetry.addData("Known distance", "%.0f cm", knownDistanceCm)
            telemetry.addData("Printed tag width", "%.1f mm", tagWidthMm)
            telemetry.addLine("----------------------------------------")
            showCamera("UP / high cam", up, VisionTuning.UP_CAM_WIDTH, upCfg.focalLengthPx, "VisionConfig.kt: UP_CAMERA.focalLengthPx")
            telemetry.addLine("----------------------------------------")
            showCamera("FRONT / ground cam", front, VisionTuning.FRONT_CAM_WIDTH, frontCfg.focalLengthPx, "VisionConfig.kt: FRONT_CAMERA.focalLengthPx")
            telemetry.addLine("----------------------------------------")
            showHeld()
            telemetry.update()
        }

        upPortal.close()
        frontPortal.close()
    }

    // ---------------- helpers ----------------

    private fun tagProcessor() = AprilTagProcessor.Builder()
        .setDrawTagOutline(true)
        .setDrawAxes(true)
        .setDrawCubeProjection(false)
        .build()

    private fun portal(
        cam: WebcamName,
        width: Int,
        height: Int,
        viewId: Int,
        processor: AprilTagProcessor,
        overlay: CalibrationOverlay,
    ): VisionPortal = VisionPortal.Builder()
        .setCamera(cam)
        .setCameraResolution(Size(width, height))
        .setLiveViewContainerId(viewId)
        .addProcessors(processor, overlay)
        .build()

    private fun handleControls() {
        val step = if (gamepad1.x) 10.0 else 1.0
        if (gamepad1.dpad_up) {
            knownDistanceCm = (knownDistanceCm + step).coerceIn(10.0, 999.0)
            sleep(120)
        } else if (gamepad1.dpad_down) {
            knownDistanceCm = (knownDistanceCm - step).coerceIn(10.0, 999.0)
            sleep(120)
        }
        if (gamepad1.dpad_right) {
            tagWidthMm = (tagWidthMm + step).coerceIn(20.0, 500.0)
            sleep(120)
        } else if (gamepad1.dpad_left) {
            tagWidthMm = (tagWidthMm - step).coerceIn(20.0, 500.0)
            sleep(120)
        }
        val a = gamepad1.a
        if (a && !prevA) {
            holding = !holding
            if (holding) held.clear()
        }
        prevA = a
    }

    /** Largest single tag in view, with the implied focal length. Clusters are ignored. */
    private fun measure(processor: AprilTagProcessor): TagMeasure? {
        val detections = try {
            processor.detections.toList()
        } catch (_: Exception) {
            emptyList()
        }
        var best: TagMeasure? = null
        var bestArea = 0.0
        for (d in detections) {
            if (d !is AprilTagSingleDetection) continue
            val corners = d.corners ?: continue
            if (corners.size < 4) continue
            val w = corners.maxOf { it.x } - corners.minOf { it.x }
            val h = corners.maxOf { it.y } - corners.minOf { it.y }
            if (w <= 1.0 || h <= 1.0) continue
            if (w * h <= bestArea) continue
            bestArea = w * h
            best = TagMeasure(
                id = d.id,
                centerPx = d.center ?: Point(0.0, 0.0),
                widthPx = w,
                heightPx = h,
                focalPx = w * (knownDistanceCm * 10.0) / tagWidthMm,
            )
        }
        return best
    }

    private fun showCamera(name: String, m: TagMeasure?, camWidth: Int, configuredFocal: Double, writeTarget: String) {
        if (m == null) {
            telemetry.addData(name, "no single tag in view")
            return
        }
        val distWithCfgCm = configuredFocal * tagWidthMm / m.widthPx / 10.0
        val hFovDeg = 2 * Math.toDegrees(atan(camWidth / 2.0 / m.focalPx))
        telemetry.addData(
            "$name tag ${m.id}",
            String.format(Locale.US, "%.0fx%.0f px, center (%.0f, %.0f)", m.widthPx, m.heightPx, m.centerPx.x, m.centerPx.y),
        )
        telemetry.addData(
            "$name LIVE focal",
            String.format(Locale.US, "%.0f px  (configured %.0f, %+.1f%%)", m.focalPx, configuredFocal, (m.focalPx / configuredFocal - 1.0) * 100.0),
        )
        telemetry.addData(
            "$name check",
            String.format(Locale.US, "dist w/ cfg = %.0f cm (actual %.0f), hFOV %.1f deg", distWithCfgCm, knownDistanceCm, hFovDeg),
        )
        telemetry.addData("$name write to", writeTarget)
    }

    private fun showHeld() {
        if (holding) {
            telemetry.addData("HOLD", "averaging... %d samples", held.size)
        } else if (held.isNotEmpty()) {
            val avg = held.average()
            val std = sqrt(held.sumOf { (it - avg) * (it - avg) } / held.size)
            telemetry.addData("HELD AVERAGE", String.format(Locale.US, "%.0f px  (%d samples, +/- %.1f)", avg, held.size, std))
            telemetry.addLine("^^ write this number into VisionConfig.kt")
        } else {
            telemetry.addData("HOLD", "press A to start averaging")
        }
    }

    private fun overlayText(name: String, m: TagMeasure?, configuredFocal: Double): String {
        if (m == null) return "$name\nno tag in view"
        return String.format(
            Locale.US,
            "%s\nfocal ~%.0f px (cfg %.0f)\ntag w=%.0fpx",
            name, m.focalPx, configuredFocal, m.widthPx,
        )
    }
}

// ============================================================================
// ============ Flywheel Calibration (was FlywheelCalibration.kt) ============
// ============================================================================


/**
 * ============ FLYWHEEL CALIBRATION — measure the shooter curve ============
 *
 * Walks the dual flywheel through 6000/5000/4000/3000/2000/1000 RPM while you
 * film each shot with a camera (a phone is fine). Then run
 * tools/flywheel_analyzer.py on the video — it tracks the ball automatically
 * and prints the measured landing distance for each RPM, formatted ready to
 * paste into globals.kt.
 *
 * HOW TO USE (full walkthrough in tools/README.md):
 *  1. Aim the robot straight down a clear runway. Put the camera SIDE-ON to
 *     the ball's flight path, and lay two tape marks on the floor a known
 *     distance apart (the analyzer uses them to convert pixels -> cm).
 *  2. Load a ball, run this OpMode, pick the RPM step (dpad UP/DOWN).
 *  3. Press A to spin the flywheel up; wait until telemetry says AT SPEED.
 *  4. Press X to FIRE ONE ball through the loader; film where it lands.
 *  5. Repeat for every RPM step (shoot in the SAME order as
 *     SHOOTER_CALIBRATION_RPM: 6000 down to 1000), then run the analyzer.
 *
 * Every shot is confirmed by the flywheel RPM dip and written to the Black
 * Box — those records are what tools/shooter_refit.py uses to keep refining
 * the shooter curve from real matches.
 */
@TeleOp(name = "Flywheel Calibration", group = "Test")
class FlywheelCalibration : LinearOpMode() {

    private val rpmSteps = SHOOTER_CALIBRATION_RPM.toList()
    private var stepIdx = 0
    private var fineRpm = 0.0
    private var spinning = false

    private lateinit var flywheel: FlyWheel
    private val loader = Loader()

    // Edge-detection memory so one button press = exactly one action.
    private var lastDpadUp = false
    private var lastDpadDown = false
    private var lastDpadLeft = false
    private var lastDpadRight = false
    private var lastA = false
    private var lastB = false
    private var lastX = false

    private fun commandedRpm(): Double =
        (rpmSteps[stepIdx] + fineRpm).coerceIn(0.0, FLYWHEEL_MOTOR_MAX_RPM)

    override fun runOpMode() {
        // boot probe: this OpMode exists to test the shooter + loader — a
        // single flywheel motor is enough for bench testing.
        val hw = HardwareProbe.probe(hardwareMap)
        if (!hw.anyFlywheel || !hw.loaderServo) {
            while (!isStopRequested) {
                telemetry.addLine("!! SHOOTER HARDWARE NOT DETECTED !!")
                telemetry.addLine("Calibration needs at least one flywheel motor + the loader servo.")
                telemetry.addData("Missing", hw.missingList.joinToString(", "))
                telemetry.update()
                sleep(50) // don't busy-spin a core during INIT
            }
            return
        }

        BlackBox.start(hardwareMap.appContext, "calib")
        hw.logToBlackBox()
        flywheel = FlyWheel(hasLeft = hw.flywheelLeft, hasRight = hw.flywheelRight)
        loader.home()
        while (!isStarted && !isStopRequested) {
            telemetry.addLine("FLYWHEEL CALIBRATION (${rpmSteps.size} RPM steps)")
            telemetry.addLine("DPAD UP/DOWN: pick RPM step   DPAD LEFT/RIGHT: fine +/-25")
            telemetry.addLine("A: toggle spin   B: stop   X: FIRE one ball (loader)")
            telemetry.addData("RPM now", String.format(Locale.US, "%.0f", rpmSteps[stepIdx]))
            telemetry.addData("Curve", ShooterModel.summary())
            telemetry.update()
        }

        while (opModeIsActive()) {
            ControlFrame.tick()
            if (gamepad1.dpad_up && !lastDpadUp && stepIdx < rpmSteps.size - 1) {
                stepIdx++; fineRpm = 0.0
            }
            if (gamepad1.dpad_down && !lastDpadDown && stepIdx > 0) {
                stepIdx--; fineRpm = 0.0
            }
            if (gamepad1.dpad_right && !lastDpadRight) fineRpm += 25.0
            if (gamepad1.dpad_left && !lastDpadLeft) fineRpm -= 25.0
            if (gamepad1.a && !lastA) spinning = !spinning
            if (gamepad1.b && !lastB) spinning = false

            val rpm = commandedRpm()
            if (spinning) flywheel.spinToRpm(rpm) else flywheel.stop()

            // Fire exactly one ball: a single loader stroke, CONFIRMED by the
            // flywheel RPM dip, then logged — these records are the raw data
            // for the shooter curve (tools/shooter_refit.py + video analyzer).
            if (gamepad1.x && !lastX && spinning && !loader.busy &&
                loader.phase == LoaderPhase.HOME
            ) {
                loader.stroke()
                while (opModeIsActive() && loader.busy) {
                    ControlFrame.tick()
                    telemetry.addData("LOADER", "stroking...")
                    telemetry.update()
                }
                val detector = ShotDetector(flywheel)
                detector.arm()
                while (opModeIsActive() && detector.update() == ShotDetector.State.ARMED) {
                    ControlFrame.tick()
                    telemetry.addData(
                        "SHOT WATCH",
                        "%.0f RPM (need a %.0f RPM dip)".format(
                            detector.measuredRpm(), SHOT_DIP_THRESHOLD_RPM,
                        ),
                    )
                    telemetry.update()
                }
                loader.retract()
                BlackBox.event(
                    "calibration_shot",
                    "commandedRpm" to rpm,
                    "dipRpm" to detector.lastDipRpm,
                    "result" to detector.state.name,
                )
            }

            telemetry.addData("Commanded", String.format(Locale.US, "%.0f RPM", rpm))
            telemetry.addData(
                "Measured L/R",
                String.format(Locale.US, "%.0f / %.0f RPM", flywheel.measuredLeftRpm(), flywheel.measuredRightRpm()),
            )
            telemetry.addData("AT SPEED", flywheel.isAtSpeed(rpm))
            telemetry.addData(
                "Step", "%d/%d (fine %+.0f)".format(stepIdx + 1, rpmSteps.size, fineRpm),
            )
            telemetry.addLine(
                "FILM THIS SHOT, then note: " +
                    String.format(Locale.US, "%.0f", rpm) + " RPM -> ? cm",
            )
            telemetry.update()

            lastDpadUp = gamepad1.dpad_up
            lastDpadDown = gamepad1.dpad_down
            lastDpadLeft = gamepad1.dpad_left
            lastDpadRight = gamepad1.dpad_right
            lastA = gamepad1.a
            lastB = gamepad1.b
            lastX = gamepad1.x
        }

        flywheel.stop(breakMotor = true)
        loader.home()
        ControlFrame.forcePoll()
        BlackBox.stop()
    }
}

// ============================================================================
// ============ Drive Only — hub + motors bring-up (was DriveOnlyOpMode.kt) ============
// ============================================================================


/**
 * ============ DRIVE ONLY — the minimal bring-up OpMode ============
 *
 * Runs on NOTHING but the Robot Controller (hub) + the 4 drive motors:
 * no cameras, no IMU, no servos, no Road Runner, no vision — nothing that
 * needs extra configured hardware. This is the first thing to run when the
 * robot is freshly wired, and the fastest loop to verify motor wiring,
 * direction and encoder sign.
 *
 * Controls:
 *   LEFT STICK drive / RIGHT STICK turn   (mecanum, same math as teleop)
 *   LEFT BUMPER (hold)   slow mode (30% power)
 *   dpad UP/DOWN         nudge forward/back a few cm (encoder position moves)
 *
 * Telemetry shows live wheel positions + velocities so you can confirm each
 * motor spins the right way and its encoder counts up.
 */
@TeleOp(name = "Drive Only (hub + motors)", group = "Test")
class DriveOnlyOpMode : LinearOpMode() {

    override fun runOpMode() {
        // boot probe: this OpMode touches ONLY the 4 drive motors — if even
        // those are missing, say so instead of crashing.
        val hw = HardwareProbe.probe(hardwareMap)
        if (!hw.driveMotors) {
            while (!isStopRequested) {
                telemetry.addLine("!! DRIVE MOTORS NOT DETECTED !!")
                telemetry.addLine("Check the Driver Station configuration.")
                telemetry.update()
                sleep(50) // don't busy-spin a core during INIT
            }
            return
        }

        // Touches ONLY frontleft/frontright/backleft/backright.
        val drivetrain = Drivetrain()

        telemetry.addLine("DRIVE ONLY: hub + 4 drive motors, nothing else.")
        telemetry.addLine("LEFT STICK drive / RIGHT STICK turn")
        telemetry.addLine("LB slow mode | dpad UP/DOWN encoder nudge")
        telemetry.update()
        waitForStart()

        while (opModeIsActive()) {
            ControlFrame.tick()
            val forward = -gamepad1.left_stick_y.toDouble()
            val strafe = gamepad1.left_stick_x.toDouble()
            val turn = gamepad1.right_stick_x.toDouble()
            val scale = if (gamepad1.left_bumper) 0.3 else 1.0

            if (gamepad1.dpad_up || gamepad1.dpad_down) {
                // small relative encoder move — also proves position control
                val cm = if (gamepad1.dpad_up) 5.0 else -5.0
                drivetrain.driveForward(cm, speed = 0.25)
                sleep(400)
            } else {
                drivetrain.driveTeleOp(forward * scale, strafe * scale, turn * scale)
            }

            val motors = listOf(
                "FL" to drivetrain.FrontLeft, "FR" to drivetrain.FrontRight,
                "BL" to drivetrain.BackLeft, "BR" to drivetrain.BackRight,
            )
            for (name in motors) {
                val (label, m) = name
                val vel = try {
                    m.encoderVelocity.into(dev.nextftc.units.RotationsPerMinute)
                } catch (_: Exception) {
                    0.0
                }
                val pos = try {
                    m.encoderPosition.into(dev.nextftc.units.Radians)
                } catch (_: Exception) {
                    0.0
                }
                telemetry.addData(
                    label,
                    String.format(Locale.US, "%.0f RPM, %.1f rad", vel, pos),
                )
            }
            telemetry.addData("slow", if (gamepad1.left_bumper) "ON" else "off")
            telemetry.update()
        }

        drivetrain.driveTeleOp(0.0, 0.0, 0.0) // stop
        ControlFrame.forcePoll()
    }
}
// ============================================================================
// ============ Neural Net Test (was NnTestOpMode.kt) ============
// ============================================================================


/**
 * ============ NEURAL NET TEST — trust-building bench ============
 *
 * Runs both on-robot neural nets against the live cameras WITHOUT any robot
 * motion, so you can validate them on the bench:
 *
 *  * the ball DETECTOR's verdicts (boxes are also drawn cyan on the DS
 *    preview by the processor itself), with the current authority mode,
 *  * the SHOT PREDICTOR's P(hit) across a range of distances at the RPM the
 *    calibrated curve commands, at the live battery voltage.
 *
 * Interpretation: the predictor table should look like a sensible curve —
 * high P(hit) where the shooter is dialed in, dropping off at the extremes.
 * If it's flat or weird, the training data or model is bad; retrain.
 */
@TeleOp(name = "Neural Net Test", group = "Test")
class NnTestOpMode : LinearOpMode() {

    override fun runOpMode() {
        // boot probe: without the cameras this OpMode has nothing to test
        val hw = HardwareProbe.probe(hardwareMap)
        if (!hw.vision) {
            while (!isStopRequested) {
                telemetry.addLine("!! AUTONOMOUS HARDWARE NOT DETECTED !!")
                telemetry.addLine("Neural Net Test needs both cameras — nothing to test.")
                telemetry.addData("Missing", hw.missingList.joinToString(", "))
                telemetry.update()
                sleep(50) // don't busy-spin a core during INIT
            }
            return
        }

        ShotPredictor.ensureLoaded(hardwareMap.appContext)
        val ballLocator = BallLocator()
        val detector = DetectorProcessor(hardwareMap.appContext)
        val vision = VisionSubsystem(
            hardwareMap,
            extraUpProcessors = emptyList(),
            extraFrontProcessors = ballLocator.processors + detector,
        )

        telemetry.addData("Detector", if (detector.loaded) "model loaded" else "no model file")
        telemetry.addData("Detector mode", NN_DETECTOR_MODE.name)
        telemetry.addData("Shot predictor", if (ShotPredictor.available) "loaded" else "not loaded")
        telemetry.update()
        waitForStart()

        while (opModeIsActive()) {
            vision.periodic()

            val targets = ballLocator.visibleTargets()
            val nn = NeuralReferee.latest()

            telemetry.addData("NN mode", NN_DETECTOR_MODE.name)
            telemetry.addData(
                "NN detections",
                "%d %s".format(
                    nn.size,
                    if (NeuralReferee.fresh()) "(fresh)" else "(stale)",
                ),
            )
            for (d in nn) {
                telemetry.addData(
                    "  NN %s".format(d.label),
                    String.format(Locale.US, "%.0f%% @ (%.0f,%.0f)", d.confidence * 100, d.x, d.y),
                )
            }
            telemetry.addData("HSV targets (post-referee)", targets.size)
            for (t in targets.take(4)) {
                telemetry.addData(
                    "  HSV %s".format(t.label),
                    String.format(Locale.US, "%.0fcm %+.1fdeg", t.distanceCm, t.bearingDeg),
                )
            }

            if (ShotPredictor.available) {
                telemetry.addData("Battery", "%.2fV".format(ShotPredictor.batteryVolts()))
                for (distCm in listOf(90.0, 110.0, 130.0, 150.0)) {
                    val rpm = ShooterModel.rpmForDistance(distCm)
                    val p = ShotPredictor.pHit(distCm, rpm, ShotPredictor.batteryVolts())
                    telemetry.addData(
                        "P(hit) %dcm".format(distCm.toInt()),
                        "rpm %.0f -> %s".format(rpm, p?.let { "%.0f%%".format(it * 100) } ?: "?"),
                    )
                }
            }
            telemetry.update()
        }

        vision.close()
    }
}
// ============================================================================
// ============ Experiment + CustomTeleOp (was main.kt) ============
// NOTE: there are intentionally NO top-level hardware singletons anymore.
// NextMotor binds itself to the shared NextFTC event loop at construction,
// so eagerly constructing motors for devices that aren't configured would
// crash the app on a background thread (e.g. a one-wheel-flywheel test
// robot). Every OpMode now constructs exactly what its HardwareProbe says
// exists — see hardware.kt (HardwareProbe).
// ============================================================================
@TeleOp(name = "Experiment (test bench)", group = "Test")
class Experiment : LinearOpMode() {
    override fun runOpMode() { // setup, code runs once
        // boot probe: no drive motors configured -> nothing to run
        val hw = HardwareProbe.probe(hardwareMap)
        if (!hw.driveMotors) {
            while (!isStopRequested) {
                telemetry.addLine("!! DRIVE MOTORS NOT DETECTED !!")
                telemetry.addLine("Check the Driver Station configuration.")
                telemetry.update()
                sleep(50)
            }
            return
        }

        // Local, probe-gated instances — see the note above the file.
        val drivetrain = Drivetrain()
        val teleOp = CustomTeleOp(drivetrain)
        val loader = Loader().also { it.home() }

        telemetry.addLine("EXPERIMENT: waits for START, then runs the drive sequence.")
        telemetry.update()
        // Never move during INIT — the sequence only runs once the match/driver
        // has actually started the OpMode.
        waitForStart()

        drivetrain.driveForward(10.0)
        sleepPolling(5000)
        drivetrain.driveBackward(100.0)
        sleepPolling(5000)
        drivetrain.driveRight(100.0)
        sleepPolling(5000)
        drivetrain.driveLeft(100.0)
        sleepPolling(5000)
        drivetrain.turn(90.0)
        sleepPolling(5000)
        // Test new driveAtAngle function
        drivetrain.driveAtAngle(50.0, 45.0) // Diagonal 45 deg for 50 cm
        sleepPolling(5000)
        drivetrain.driveAtAngle(60.0, 15.0) // 15 deg for 60 cm
        sleepPolling(5000)
        drivetrain.flushMotors()
        ControlFrame.forcePoll()

        while (opModeIsActive()) { // main loop
            ControlFrame.tick()
            // Use THIS OpMode's gamepad1: the old global `controller` captured
            // BlocksOpModeCompanion.gamepad1 at class-load time, which is only
            // ever populated by the Blocks runtime and never updates for a
            // Kotlin OpMode (null -> guaranteed NPE, or a dead stale object).
            teleOp.refresh(gamepad1)

            // Shooter/loader test bench: X (hold) indexes one ball per stroke
            // into the (possibly spinning) flywheel, Y (tap) homes the loader.
            if (gamepad1.x) loader.stroke()
            if (gamepad1.y) loader.home()
            telemetry.addData("LOADER", "phase %s, %d strokes".format(loader.phase.name, loader.strokeCount))
            telemetry.update()
        }
    }
}
/**
 * Minimal gamepad-to-drivetrain adapter used by the [Experiment] OpMode
 * (merged here from the old TeleOP.kt — it belongs with the robot singletons).
 */
class CustomTeleOp(
    private val drivetrain: Drivetrain
) {
    fun refresh(gamepad: Gamepad) {
        val forward = -gamepad.left_stick_y.toDouble()
        val strafe = gamepad.left_stick_x.toDouble()
        val turn = gamepad.right_stick_x.toDouble()

        drivetrain.driveTeleOp(forward, strafe, turn)
    }
}

/** Sleeps while polling the NextFTC motor event loop so position/velocity
 *  setpoints keep updating during blind waits. */
private fun LinearOpMode.sleepPolling(ms: Long) {
    val end = System.currentTimeMillis() + ms
    while (System.currentTimeMillis() < end) {
        ControlFrame.tick()
        sleep(10)
    }
    ControlFrame.forcePoll()
}

/**
 * ============ DRIVETRAIN SETUP & VERIFY — the bring-up bench ============
 *
 * Verifies the DrivetrainSpec measurements BEFORE trusting any position move.
 * Everything printed derives from the spec — you bring a tape measure.
 *
 *  INIT: prints every derived constant (ticks/rev, cm-per-tick, RR params).
 *  A (tap):  spin all drive wheels at 0.25 power for 1s -> reports each
 *            motor's encoder ticks + sign (verify ticks match the spec and
 *            fix `direction` in motors.kt for any motor that runs backward).
 *  B (tap):  commands a 50 cm straight crawl -> shows the exact tick count
 *            to expect; tape-measure the result and compare.
 *  X (hold): LIVE encoder readout (drive motors + dead wheels). Push the
 *            robot by hand: forward 1 m -> par0/par1 should both read
 *            POSITIVE (~spec) ticks; strafe left 1 m -> perp reads POSITIVE.
 *            If any pod counts the WRONG WAY, flip its *_REVERSED flag in
 *            DrivetrainSpec (never rewire the hub).
 */
@TeleOp(name = "Drivetrain Setup & Verify", group = "Test")
class DrivetrainSetupOpMode : LinearOpMode() {

    override fun runOpMode() {
        val drivetrain = Drivetrain()
        val frontLeft = hardwareMap.get(DcMotorEx::class.java, DrivetrainFrontLeft)
        val frontRight = hardwareMap.get(DcMotorEx::class.java, DrivetrainFrontRight)
        val backLeft = hardwareMap.get(DcMotorEx::class.java, DrivetrainBackLeft)
        val backRight = hardwareMap.get(DcMotorEx::class.java, DrivetrainBackRight)
        fun deadWheel(name: String): DcMotorEx? = try {
            hardwareMap.get(DcMotorEx::class.java, name)
        } catch (_: Exception) {
            null
        }
        val par0 = deadWheel(DEAD_WHEEL_PAR0_NAME)
        val par1 = deadWheel(DEAD_WHEEL_PAR1_NAME)
        val perp = deadWheel(DEAD_WHEEL_PERP_NAME)

        fun snapshot(): List<Int> = listOf(
            frontLeft.currentPosition, frontRight.currentPosition,
            backLeft.currentPosition, backRight.currentPosition,
            par0?.currentPosition ?: 0, par1?.currentPosition ?: 0, perp?.currentPosition ?: 0,
        )

        for (pair in DrivetrainSpec.summary()) {
            telemetry.addData(pair.first, pair.second)
        }
        telemetry.addData("Dead wheels", if (par0 != null && par1 != null && perp != null) "configured" else "not configured")
        telemetry.addLine("A: spin test 1s | B: 50cm crawl | X(hold): live encoders")
        telemetry.update()
        waitForStart()

        var lastA = false
        var lastB = false
        var base = snapshot()

        while (opModeIsActive()) {
            // Position/velocity moves through NextFTC need the event loop polled.
            ControlFrame.tick()
            val now = snapshot()

            // ---- A: 1-second spin test ----
            if (gamepad1.a && !lastA) {
                base = snapshot()
                for (m in listOf(frontLeft, frontRight, backLeft, backRight)) m.power = 0.25
                sleep(1000)
                for (m in listOf(frontLeft, frontRight, backLeft, backRight)) m.power = 0.0
                val after = snapshot()
                val names = listOf("FL", "FR", "BL", "BR")
                for (i in 0..3) {
                    val ticks = after[i] - base[i]
                    val expectedSign = if (i == 1 || i == 3) "-" else "+" // FR/BR face backward
                    telemetry.addData(
                        "${names[i]} spin",
                        "%+d ticks (%s fwd expected; ~%.0f%% of %.0f t/rev)".format(
                            ticks, expectedSign,
                            kotlin.math.abs(ticks).toDouble() / DrivetrainSpec.driveTicksPerWheelRev * 100.0,
                            DrivetrainSpec.driveTicksPerWheelRev,
                        ),
                    )
                }
                telemetry.update()
                sleep(3000)
            }

            // ---- B: 50 cm straight crawl ----
            if (gamepad1.b && !lastB) {
                base = snapshot()
                drivetrain.driveForward(50.0, 0.25)
                // Poll while waiting, or the position controller never moves the motors.
                val crawlEnd = System.currentTimeMillis() + 4000
                while (System.currentTimeMillis() < crawlEnd) {
                    ControlFrame.tick()
                    sleep(20)
                }
                drivetrain.flushMotors()
                ControlFrame.forcePoll()
                val after = snapshot()
                telemetry.addData(
                    "50cm crawl",
                    "expected %+d ticks/wheel (FL measured %+d)".format(
                        DrivetrainSpec.cmToWheelTicks(50.0), after[0] - base[0],
                    ),
                )
                telemetry.addLine("Tape-measure: did the robot move 50 cm?")
                telemetry.update()
                sleep(3000)
            }

            lastA = gamepad1.a
            lastB = gamepad1.b

            // ---- X (hold): live encoder readout for hand pushes ----
            if (gamepad1.x) {
                telemetry.addData("FL/FR/BL/BR", "%d %d %d %d".format(now[0], now[1], now[2], now[3]))
                telemetry.addData(
                    "par0/par1/perp",
                    "%d %d %d%s".format(
                        now[4], now[5], now[6],
                        if (par0 == null) " (pods not configured)" else "",
                    ),
                )
                telemetry.addData(
                    "1m push should read",
                    "par0/par1 ~%.0f ticks; perp ~%.0f ticks (STRAFE 1m)".format(
                        100.0 / DrivetrainSpec.driveCmPerTick, // rough guide via drive ticks
                        100.0 / 2.54 / DrivetrainSpec.deadWheelInPerTick,
                    ),
                )
                telemetry.addData("par0/par1 (1m fwd)", "~%.0f ticks".format(100.0 / 2.54 / DrivetrainSpec.deadWheelInPerTick))
                telemetry.update()
            } else {
                for (pair in DrivetrainSpec.summary()) telemetry.addData(pair.first, pair.second)
                telemetry.addData("Dead wheels", if (par0 != null && par1 != null && perp != null) "configured" else "not configured")
                telemetry.addLine("A: spin test 1s | B: 50cm crawl | X(hold): live encoders")
                telemetry.update()
            }
        }

        drivetrain.flushMotors()
        ControlFrame.forcePoll()
    }
}
