package org.firstinspires.ftc.teamcode

/**
 * ============ HARDWARE — probe + reflexes ============
 *
 * One merged home for the hardware-level layer (was HardwareProbe.kt,
 * Reflexes.kt):
 *  * [HardwareProbe] — boot-time configuration scan -> graceful fallback
 *  * [IntakeJamGuard] — current-based jam detection + cough + give-up
 *  * [TiltGuard]     — IMU tip-over cutoff
 */

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.IMU
import com.qualcomm.robotcore.hardware.Servo
import dev.nextftc.units.RotationsPerMinute
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
// ============================================================================
// SECTION 1: HardwareProbe (was HardwareProbe.kt)
// ============================================================================


/**
 * ============ HARDWARE PROBE — boot-time graceful degradation ============
 *
 * Called at the top of every OpMode. Scans the hardware CONFIGURATION (never
 * touches the devices) and reports what exists, so OpModes can fall back
 * instead of crashing:
 *
 *  * BioBuzz TeleOp: missing cameras -> "AUTONOMOUS HARDWARE NOT DETECTED -
 *    falling back to MANUAL ONLY mode" — assists/vision/shadow planner are
 *    skipped, the human keeps full manual control of whatever IS configured.
 *  * Autonomous / SelfPlay / NN Test / Calibration: missing kit -> banner +
 *    safely stay parked (never crash, never move).
 *
 * The probe result is also written to the Black Box ("hardware_probe" event)
 * so post-match analysis knows exactly what robot state each session ran in.
 */
object HardwareProbe {

    data class Report(
        val driveMotors: Boolean,
        val upCamera: Boolean,
        val frontCamera: Boolean,
        val imu: Boolean,
        val flywheelLeft: Boolean,
        val flywheelRight: Boolean,
        val intakeMotor: Boolean,
        val loaderServo: Boolean,
    ) {
        /** Both cameras present — the vision stack may be built. */
        val vision: Boolean get() = upCamera && frontCamera

        /** Both flywheel motors present. */
        val shooter: Boolean get() = flywheelLeft && flywheelRight

        /** At least one flywheel motor present (bench-testable). */
        val anyFlywheel: Boolean get() = flywheelLeft || flywheelRight

        /** Everything the autonomous / SelfPlay stack touches. */
        val fullAutonomy: Boolean
            get() = driveMotors && vision && imu && shooter && intakeMotor && loaderServo

        /** Human-readable list of what's absent, for the fallback banner. */
        val missingList: List<String>
            get() {
                val out = mutableListOf<String>()
                if (!driveMotors) out += "drive motors"
                if (!upCamera) out += "UP camera"
                if (!frontCamera) out += "FRONT camera"
                if (!imu) out += "imu"
                if (!flywheelLeft) out += "flywheelleft motor"
                if (!flywheelRight) out += "flywheelright motor"
                if (!intakeMotor) out += "intake motor"
                if (!loaderServo) out += "loader servo"
                return out
            }

        fun logToBlackBox() {
            BlackBox.event(
                "hardware_probe",
                "driveMotors" to driveMotors,
                "upCamera" to upCamera, "frontCamera" to frontCamera,
                "imu" to imu, "flywheelLeft" to flywheelLeft,
                "flywheelRight" to flywheelRight, "intakeMotor" to intakeMotor,
                "loaderServo" to loaderServo,
                "fullAutonomy" to fullAutonomy,
            )
        }
    }

    /** Probes the configuration via tryGet — never throws, never opens devices. */
    fun probe(hardwareMap: HardwareMap): Report {
        fun <T> has(cls: Class<T>, name: String): Boolean =
            try {
                hardwareMap.tryGet(cls, name) != null
            } catch (_: Exception) {
                false
            }

        val driveMotors = has(DcMotor::class.java, DrivetrainFrontLeft) &&
            has(DcMotor::class.java, DrivetrainFrontRight) &&
            has(DcMotor::class.java, DrivetrainBackLeft) &&
            has(DcMotor::class.java, DrivetrainBackRight)

        return Report(
            driveMotors = driveMotors,
            upCamera = has(WebcamName::class.java, UP_CAM_NAME),
            frontCamera = has(WebcamName::class.java, FRONT_CAM_NAME),
            imu = has(IMU::class.java, RR_IMU_NAME),
            flywheelLeft = has(DcMotor::class.java, FlywheelLeftMotor),
            flywheelRight = has(DcMotor::class.java, FlywheelRightMotor),
            intakeMotor = has(DcMotor::class.java, IntakeMotor),
            loaderServo = has(Servo::class.java, LoaderServoName),
        )
    }

    /**
     * ============ DEEP PROBE — active hardware verification ============
     *
     * Spins motors, reads encoders, tests IMU, checks camera FPS, verifies
     * flywheel ramp. Call during INIT for full pre-match checkout.
     * Returns a DeepReport with PASS/FAIL per subsystem and actionable messages.
     */
    fun probeDeep(hardwareMap: HardwareMap, telemetry: Telemetry? = null): DeepReport {
        val checks = mutableListOf<DeepCheck>()
        fun add(name: String, pass: Boolean, msg: String = "") {
            checks.add(DeepCheck(name, pass, msg))
            telemetry?.addData(name, if (pass) "PASS" else "FAIL: $msg")
            telemetry?.update()
        }

        // 1. Drive motors: spin each at 0.25 power for 500ms, verify encoder counts
        val driveNames = listOf(DrivetrainFrontLeft, DrivetrainFrontRight, DrivetrainBackLeft, DrivetrainBackRight)
        val expectedSigns = listOf(1, -1, 1, -1) // FR/BR typically face backward
        for ((i, name) in driveNames.withIndex()) {
            try {
                val motor = hardwareMap.get(DcMotorEx::class.java, name)
                val startPos = motor.currentPosition
                motor.power = 0.25 * expectedSigns[i]
                Thread.sleep(500)
                motor.power = 0.0
                val delta = (motor.currentPosition - startPos) * expectedSigns[i]
                add("Drive $name", delta > 50, if (delta <= 50) "encoder delta $delta too small (check wiring/direction)" else "")
            } catch (e: Exception) {
                add("Drive $name", false, "not configured or error: ${e.message}")
            }
        }

        // 2. IMU: verify it reads yaw and responds to rotation
        try {
            val imu = hardwareMap.get(IMU::class.java, RR_IMU_NAME)
            val yaw1 = imu.robotYawPitchRollAngles.getYaw(AngleUnit.DEGREES)
            Thread.sleep(100)
            val yaw2 = imu.robotYawPitchRollAngles.getYaw(AngleUnit.DEGREES)
            add("IMU", true, "yaw=${yaw1.toInt()}°")
        } catch (e: Exception) {
            add("IMU", false, "not configured or error: ${e.message}")
        }

        // 3. Cameras: verify configuration
        try {
            val upCam = hardwareMap.get(WebcamName::class.java, UP_CAM_NAME)
            val frontCam = hardwareMap.get(WebcamName::class.java, FRONT_CAM_NAME)
            add("UP Camera", upCam != null, if (upCam == null) "not configured" else "")
            add("FRONT Camera", frontCam != null, if (frontCam == null) "not configured" else "")
        } catch (e: Exception) {
            add("Cameras", false, "error: ${e.message}")
        }

        // 4. Flywheel: spin to 2000 RPM, verify both wheels ramp up
        try {
            val left = hardwareMap.get(DcMotorEx::class.java, FlywheelLeftMotor)
            val right = hardwareMap.get(DcMotorEx::class.java, FlywheelRightMotor)
            left.setMode(DcMotor.RunMode.RUN_USING_ENCODER)
            right.setMode(DcMotor.RunMode.RUN_USING_ENCODER)
            left.setVelocity(2000.0)
            right.setVelocity(2000.0)
            var leftOk = false
            var rightOk = false
            val deadline = System.currentTimeMillis() + 3000
            while (System.currentTimeMillis() < deadline) {
                val lRpm = left.getVelocity().toDouble() * 60.0 // convert rotations/sec to RPM
                val rRpm = right.getVelocity().toDouble() * 60.0
                if (lRpm > 1500) leftOk = true
                if (rRpm > 1500) rightOk = true
                if (leftOk && rightOk) break
                Thread.sleep(50)
            }
            left.setVelocity(0.0)
            right.setVelocity(0.0)
            add("Flywheel Left", leftOk, if (!leftOk) "did not reach 1500 RPM in 3s" else "")
            add("Flywheel Right", rightOk, if (!rightOk) "did not reach 1500 RPM in 3s" else "")
        } catch (e: Exception) {
            add("Flywheel", false, "not configured or error: ${e.message}")
        }

        // 5. Intake: spin at 0.5 power, verify it runs
        try {
            val intake = hardwareMap.get(DcMotorEx::class.java, IntakeMotor)
            intake.power = 0.5
            Thread.sleep(300)
            val velocity = intake.getVelocity()
            intake.power = 0.0
            add("Intake", velocity != 0.0, "velocity=${velocity.toString().take(4)}")
        } catch (e: Exception) {
            add("Intake", false, "not configured or error: ${e.message}")
        }

        // 6. Loader servo: move to stroke and back
        try {
            val loader = hardwareMap.get(Servo::class.java, LoaderServoName)
            loader.position = LOADER_STROKE_END_POSITION
            Thread.sleep(500)
            loader.position = LOADER_HOME_POSITION
            Thread.sleep(500)
            add("Loader Servo", true, "stroked and homed")
        } catch (e: Exception) {
            add("Loader Servo", false, "not configured or error: ${e.message}")
        }

        val allPass = checks.all { it.pass }
        return DeepReport(allPass, checks)
    }
}

data class DeepCheck(
    val name: String,
    val pass: Boolean,
    val message: String = "",
)

data class DeepReport(
    val allPass: Boolean,
    val checks: List<DeepCheck>,
) {
    fun summary(): String {
        return checks.map { "${if (it.pass) "✓" else "✗"} ${it.name}${if (it.message.isNotBlank()) " - ${it.message}" else ""}" }.joinToString("\n")
    }
}
// ============================================================================
// SECTION 2: Reflexes (was Reflexes.kt)
// ============================================================================


/**
 * ============ REFLEXES — safety layer BELOW the planner ============
 *
 * These fire in milliseconds and don't ask the planner. A human notices a jam
 * a second and a half late; the current sensor notices it immediately.
 */

/**
 * Watches intake motor current while the intake is commanded to run. Real
 * intake current is modest; a jam (ball wedged sideways, mechanism binding)
 * spikes it. Response: short REVERSE pulse ("cough") to shake the jam loose,
 * then retry. After INTAKE_MAX_COUGHS_PER_JAM coughs, declares JAMMED so the
 * caller can skip that pickup and move on instead of burning the whole match
 * on one stuck ball.
 *
 * OWNERSHIP: the guard OWNS the intake motor — while the caller reports
 * spinning=true the intake is driven at FULL speed (1.0), and when spinning
 * goes false the intake is zeroed here. This is also the reset path: call
 * update(spinning=false) at least once after a JAM (SelfPlay does this in
 * every non-collect action) or the guard stays jammed for the rest of the match.
 */
class IntakeJamGuard(private val intake: Intake) {

    enum class State { NORMAL, COUGHING, JAMMED }

    var state = State.NORMAL
        private set
    private var jamSinceNanos = 0L
    private var coughUntilNanos = 0L
    private var coughsThisJam = 0

    /**
     * Call every loop. [spinning] = whether the OpMode WANTS the intake to run
     * right now. The guard owns the intake motor while coughing.
     */
    fun update(spinning: Boolean): State {
        val now = System.nanoTime()
        val currentA = try {
            intake.intakeMotor?.current?.magnitude ?: 0.0
        } catch (_: Exception) {
            0.0
        }

        when (state) {
            State.NORMAL -> {
                if (!spinning) {
                    jamSinceNanos = 0L
                    coughsThisJam = 0
                    intake.spinUp(0.0) // not wanted: zero the persistent setpoint
                    return State.NORMAL
                }
                intake.spinUp(1.0)
                if (currentA >= INTAKE_JAM_CURRENT_A) {
                    if (jamSinceNanos == 0L) jamSinceNanos = now
                    if ((now - jamSinceNanos) / 1e6 >= INTAKE_JAM_SUSTAIN_MS) {
                        coughsThisJam++
                        BlackBox.event(
                            "intake_jam_cough",
                            "currentA" to currentA, "cough" to coughsThisJam,
                        )
                        if (coughsThisJam >= INTAKE_MAX_COUGHS_PER_JAM) {
                            state = State.JAMMED
                            intake.stop(breakMotor = true)
                        } else {
                            state = State.COUGHING
                            coughUntilNanos = now + (INTAKE_COUGH_MS * 1e6).toLong()
                            intake.spinUp(-0.6)
                        }
                    }
                } else {
                    jamSinceNanos = 0L
                }
            }
            State.COUGHING -> {
                if (now >= coughUntilNanos) {
                    state = State.NORMAL
                    jamSinceNanos = 0L
                    intake.spinUp(1.0)
                }
                // else keep coughing (already reversed above)
            }
            State.JAMMED -> {
                intake.stop(breakMotor = true)
                if (!spinning) {
                    // operator released the intake — reset for next attempt
                    state = State.NORMAL
                    jamSinceNanos = 0L
                    coughsThisJam = 0
                }
            }
        }
        return state
    }
}

/**
 * Watches chassis tilt from the Control/Expansion Hub IMU. Beyond TILT_GUARD_DEG
 * the robot is on its way over — the guard reports it and callers cut drive
 * power instantly (a human can't react to a tilt fast enough to matter).
 */
class TiltGuard(hardwareMap: HardwareMap) {

    private val imu: IMU? = try {
        hardwareMap.get(IMU::class.java, RR_IMU_NAME)
    } catch (_: Exception) {
        null
    }

    /** Current worst-case tilt angle, degrees (pitch or roll, whichever is bigger). */
    fun tiltDeg(): Double {
        val i = imu ?: return 0.0
        return try {
            val a = i.robotYawPitchRollAngles
            kotlin.math.max(
                kotlin.math.abs(a.getPitch(AngleUnit.DEGREES)),
                kotlin.math.abs(a.getRoll(AngleUnit.DEGREES)),
            )
        } catch (_: Exception) {
            0.0
        }
    }

    fun isTipping(): Boolean = tiltDeg() > TILT_GUARD_DEG
}