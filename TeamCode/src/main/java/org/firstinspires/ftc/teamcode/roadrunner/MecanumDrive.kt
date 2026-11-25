package org.firstinspires.ftc.teamcode.roadrunner

import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.acmerobotics.roadrunner.AccelConstraint
import com.acmerobotics.roadrunner.Action
import com.acmerobotics.roadrunner.now
import com.acmerobotics.roadrunner.range
import com.acmerobotics.roadrunner.AngularVelConstraint
import com.acmerobotics.roadrunner.DualNum
import com.acmerobotics.roadrunner.HolonomicController
import com.acmerobotics.roadrunner.MecanumKinematics
import com.acmerobotics.roadrunner.MinVelConstraint
import com.acmerobotics.roadrunner.MotorFeedforward
import com.acmerobotics.roadrunner.Pose2d
import com.acmerobotics.roadrunner.PoseVelocity2d
import com.acmerobotics.roadrunner.PoseVelocity2dDual
import com.acmerobotics.roadrunner.ProfileAccelConstraint
import com.acmerobotics.roadrunner.ProfileParams
import com.acmerobotics.roadrunner.Rotation2d
import com.acmerobotics.roadrunner.Time
import com.acmerobotics.roadrunner.TimeTrajectory
import com.acmerobotics.roadrunner.TimeTurn
import com.acmerobotics.roadrunner.TrajectoryActionBuilder
import com.acmerobotics.roadrunner.TrajectoryBuilderParams
import com.acmerobotics.roadrunner.TurnConstraints
import com.acmerobotics.roadrunner.Twist2d
import com.acmerobotics.roadrunner.Twist2dDual
import com.acmerobotics.roadrunner.VelConstraint
import com.acmerobotics.roadrunner.Vector2d
import com.acmerobotics.roadrunner.ftc.DownsampledWriter
import com.acmerobotics.roadrunner.ftc.FlightRecorder
import com.acmerobotics.roadrunner.ftc.throwIfModulesAreOutdated
import com.acmerobotics.roadrunner.ftc.OverflowEncoder
import com.acmerobotics.roadrunner.ftc.RawEncoder
import com.qualcomm.hardware.lynx.LynxModule
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.IMU
import com.qualcomm.robotcore.hardware.VoltageSensor
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles
import org.firstinspires.ftc.teamcode.DeadWheelMath
import org.firstinspires.ftc.teamcode.DrivetrainSpec
import org.firstinspires.ftc.teamcode.DriveConfig
import org.firstinspires.ftc.teamcode.DEAD_WHEEL_PAR0_NAME
import org.firstinspires.ftc.teamcode.DEAD_WHEEL_PAR1_NAME
import org.firstinspires.ftc.teamcode.DEAD_WHEEL_PERP_NAME
import org.firstinspires.ftc.teamcode.DrivetrainBackLeft
import org.firstinspires.ftc.teamcode.DrivetrainBackRight
import org.firstinspires.ftc.teamcode.DrivetrainFrontLeft
import org.firstinspires.ftc.teamcode.DrivetrainFrontRight
import org.firstinspires.ftc.teamcode.RR_IMU_NAME
import org.firstinspires.ftc.teamcode.roadrunner.messages.DeadWheelLocalizerInputsMessage
import org.firstinspires.ftc.teamcode.roadrunner.messages.DriveCommandMessage
import org.firstinspires.ftc.teamcode.roadrunner.messages.MecanumCommandMessage
import org.firstinspires.ftc.teamcode.roadrunner.messages.MecanumLocalizerInputsMessage
import org.firstinspires.ftc.teamcode.roadrunner.messages.PoseMessage
import java.util.LinkedList
import kotlin.math.ceil
import kotlin.math.cos
import kotlin.math.max
import kotlin.math.sin
import kotlin.math.hypot
import kotlin.math.abs



/**
 * Road Runner 1.0 mecanum drive, ported to Kotlin from the official quickstart
 * (https://github.com/acmerobotics/road-runner-quickstart) to match this
 * project's Kotlin + NextFTC architecture. Units are INCHES and RADIANS
 * throughout Road Runner (per RR 1.0 convention), NOT centimeters.
 */
@Config
class MecanumDrive(hardwareMap: HardwareMap, initialPose: Pose2d) {

    /** Tunable parameters (editable live via FTC Dashboard once it is started). */
    class Params {
        // IMU orientation
        // TODO: fill in these values based on the physical Control/Expansion Hub mounting.
        //   see https://ftc-docs.firstinspires.org/en/latest/programming_resources/imu/imu.html
        // Hub mounted on LEFT side, pointing INWARD (logo faces RIGHT), USB ports UP
        var logoFacingDirection = RevHubOrientationOnRobot.LogoFacingDirection.RIGHT
        var usbFacingDirection = RevHubOrientationOnRobot.UsbFacingDirection.UP

        // drive model parameters (defaults COMPUTED from DrivetrainSpec's
        // physical measurements; fine-tune live via FTC Dashboard after that)
        var inPerTick = DrivetrainSpec.rrInPerTick
        var lateralInPerTick = DrivetrainSpec.rrLateralInPerTick
        var trackWidthTicks = DrivetrainSpec.rrTrackWidthTicks

        // Feedforward parameters.
        //
        // These are DERIVED from the hardware measurements in DrivetrainSpec
        // instead of being left at zero, because a zero feedforward means
        // Road Runner commands NO motor power at all — every trajectory move
        // (the autonomous park move, for one) silently does nothing. The
        // derived values get the robot driving on the first run; refine them
        // live via FTC Dashboard once there is time.
        //
        //   kV: derived from the motor's free speed (full power => full speed)
        //   kS: static friction, 5% of full power to break loose
        //   kA: acceleration term, 0 until the robot is driving cleanly
        var kS = DrivetrainSpec.RR_KS
        var kV = DrivetrainSpec.rrKV
        var kA = DrivetrainSpec.RR_KA

        // Path profile parameters (in inches)
        var maxWheelVel = 50.0
        var minProfileAccel = -30.0
        var maxProfileAccel = 50.0

        // Turn profile parameters (in radians)
        var maxAngVel = Math.PI // shared with path
        var maxAngAccel = Math.PI

        // Path controller gains.
        //
        // Road Runner's holonomic controller computes
        //     command = targetVelocity + axialGain * positionError + axialVelGain * velocityError
        // so these are "inches/sec per inch of error" and "inches/sec per
        // inch/sec of error". Zero gains means the controller contributes
        // NOTHING — the robot would coast on feedforward alone and drift.
        //
        // The defaults below are deliberately gentle: a mecanum that is
        // over-gained oscillates, and an oscillating robot is worse than one
        // that converges slowly. Tune upward only if the robot visibly lags
        // its trajectory.
        var axialGain = 2.0
        var lateralGain = 2.0
        var headingGain = 2.0 // shared with turn

        var axialVelGain = 0.5
        var lateralVelGain = 0.5
        var headingVelGain = 0.5 // shared with turn
    }

    val kinematics = MecanumKinematics(
        PARAMS.inPerTick * PARAMS.trackWidthTicks,
        PARAMS.inPerTick / PARAMS.lateralInPerTick,
    )

    val defaultTurnConstraints = TurnConstraints(
        PARAMS.maxAngVel, -PARAMS.maxAngAccel, PARAMS.maxAngAccel,
    )
    val defaultVelConstraint: VelConstraint = MinVelConstraint(
        listOf(
            kinematics.WheelVelConstraint(PARAMS.maxWheelVel),
            AngularVelConstraint(PARAMS.maxAngVel),
        ),
    )
    val defaultAccelConstraint: AccelConstraint =
        ProfileAccelConstraint(PARAMS.minProfileAccel, PARAMS.maxProfileAccel)

    val leftFront: DcMotorEx
    val leftBack: DcMotorEx
    val rightBack: DcMotorEx
    val rightFront: DcMotorEx

    val voltageSensor: VoltageSensor

    /**
     * The IMU, if it is present AND initializes successfully.
     *
     * This is deliberately EAGER and wrapped in try/catch rather than lazy.
     * Road Runner's `LazyHardwareMapImu.get()` calls `imu.initialize()`, which
     * THROWS if the IMU is present in the config but unhealthy (a loose hub
     * cable, a wrong orientation, a hub that has not finished booting). Because
     * the localizers call `get()` from their constructor, a lazy IMU meant a
     * bad IMU could crash EVERY OpMode that builds a drive — including the
     * bench tests you would use to diagnose it.
     *
     * With this, a broken IMU downgrades the robot to encoder-only heading
     * (from the dead wheels or the drive wheels) instead of killing the
     * OpMode. The robot still drives; it just has worse heading.
     *
     * Null = no usable IMU. See [imuProblem] for why.
     */
    val imu: IMU?

    /** Human-readable reason the IMU is unavailable (null when it works). */
    val imuProblem: String?

    /**
     * Zeroes the IMU yaw so heading reads 0 for the robot's CURRENT
     * orientation.
     *
     * Called automatically at construction, and again by [resetHeading]. This
     * is what makes the IMU a usable BACKUP for the dead wheels: without a
     * zero, the heading is an arbitrary angle left over from whenever the
     * hub was powered on, so a field-centric drive or an autonomous turn
     * would be off by that unknown angle. Zeroing at the start of every
     * OpMode makes heading meaningful and repeatable.
     */
    fun resetHeading() {
        try {
            imu?.resetYaw()
        } catch (_: Exception) {
            // A reset failure is not fatal — the heading is still usable, it
            // just is not zeroed.
        }
    }

    /** Pluggable localization; defaults to mecanum + IMU. Swappable for dead wheels / Pinpoint. */
    var localizer: Localizer

    /**
     * Optional external pose correction hook (the hybrid visual localization
     * system attaches here). See [PoseCorrector]. Null = pure Road Runner.
     */
    var poseCorrector: PoseCorrector? = null

    /**
     * Master safety gate: vision corrections are only allowed to move the Road
     * Runner pose when this is true AND a [PoseCorrector] is attached. Keep it
     * false until the visual test procedure (see VisionConfig.kt docs) has been
     * passed on the physical robot.
     */
    var visionCorrectionEnabled = false

    // ===== ANTI-RAMMING / PUSH DETECTION =====
    /** Last commanded velocity (set by setDrivePowers). */
    var lastCommandedVelocity: PoseVelocity2d? = null
    /** Whether an external push is currently detected. */
    var isBeingPushed: Boolean = false
    /** Nanoseconds when push was first detected (for debounce). */
    var pushDetectedNanos: Long = 0
    /** Nanoseconds when push ended (for release delay). */
    var pushReleasedNanos: Long = 0
    /** True if anchor mode was manually engaged via gamepad combo. */
    var anchorManualEngage: Boolean = false

    /**
     * Returns the velocity error (measured - commanded) for push detection.
     * Call after updatePoseEstimate() to get fresh measured velocity.
     */
    fun getVelocityError(): PoseVelocity2d? {
        val measured = lastMeasuredVelocity
        val commanded = lastCommandedVelocity
        if (measured == null || commanded == null) return null
        return PoseVelocity2d(
            Vector2d(
                measured.component1().x - commanded.component1().x,
                measured.component1().y - commanded.component1().y,
            ),
            measured.component2() - commanded.component2(),
        )
    }

    /** Most recent measured velocity from the localizer (updated in updatePoseEstimate). */
    var lastMeasuredVelocity: PoseVelocity2d? = null

    /**
     * Updates push detection state. Call this each cycle after updatePoseEstimate().
     */
    fun updatePushState() {
        if (!org.firstinspires.ftc.teamcode.PUSH_DETECTION_ENABLED) {
            isBeingPushed = false
            anchorManualEngage = false
            return
        }

        val error = getVelocityError()
        val now = System.nanoTime()
        var pushNow = false

        if (error != null) {
            val linearErr = hypot(error.component1().x, error.component1().y)
            val headingErrDeg = abs(Math.toDegrees(error.component2()))
            if (linearErr > org.firstinspires.ftc.teamcode.PUSH_VELOCITY_THRESHOLD || headingErrDeg > org.firstinspires.ftc.teamcode.PUSH_HEADING_THRESHOLD_DEG) {
                pushNow = true
            }
        }

        if (pushNow) {
            if (pushDetectedNanos == 0L) pushDetectedNanos = now
            val debounced = (now - pushDetectedNanos) / 1e6 >= org.firstinspires.ftc.teamcode.PUSH_DEBOUNCE_MS
            isBeingPushed = debounced || anchorManualEngage
            pushReleasedNanos = 0L
        } else {
            pushDetectedNanos = 0L
            if (isBeingPushed && !anchorManualEngage) {
                if (pushReleasedNanos == 0L) pushReleasedNanos = now
                val released = (now - pushReleasedNanos) / 1e6 >= org.firstinspires.ftc.teamcode.PUSH_RELEASE_MS
                if (released) isBeingPushed = false
            } else if (!anchorManualEngage) {
                isBeingPushed = false
                pushReleasedNanos = 0L
            }
        }
    }

    /** Manually engage/disengage anchor mode (e.g., via gamepad combo). */
    fun setAnchorManual(engage: Boolean) {
        anchorManualEngage = engage
        if (engage) isBeingPushed = true
    }

    private val poseHistory = LinkedList<Pose2d>()


    private val estimatedPoseWriter = DownsampledWriter("ESTIMATED_POSE", 50_000_000)
    private val targetPoseWriter = DownsampledWriter("TARGET_POSE", 50_000_000)
    private val driveCommandWriter = DownsampledWriter("DRIVE_COMMAND", 50_000_000)
    private val mecanumCommandWriter = DownsampledWriter("MECANUM_COMMAND", 50_000_000)

    /**
     * Mecanum + IMU localizer (Road Runner 1.0 quickstart algorithm, Kotlin port).
     * If IMU is available, uses it for heading.
     * If IMU is MISSING, integrates heading from mecanum kinematics omega.
     */
    inner class DriveLocalizer(
        initialPose: Pose2d,
        private val imuAvailable: Boolean,
    ) : Localizer {
        val leftFront = OverflowEncoder(RawEncoder(this@MecanumDrive.leftFront))
        val leftBack = OverflowEncoder(RawEncoder(this@MecanumDrive.leftBack))
        val rightBack = OverflowEncoder(RawEncoder(this@MecanumDrive.rightBack))
        val rightFront = OverflowEncoder(RawEncoder(this@MecanumDrive.rightFront))
        private val imu: IMU? = if (imuAvailable) this@MecanumDrive.imu else null

        init {
            // The SDK already flips the reported position for a REVERSE motor,
            // and Road Runner's RawEncoder flips it AGAIN for a REVERSE motor.
            // Flipping the ENCODER direction here cancels that second flip, so
            // the encoder value Road Runner sees is the SAME value NextFTC's
            // Drivetrain sees (SDK-adjusted): all four wheels count UP when
            // the robot moves forward. Road Runner's kinematics assumes exactly
            // that, so the odometry stays correct instead of silently driving
            // the pose sideways.
            leftFront.direction = driveEncoderDirection(DriveConfig.FL_REVERSED)
            leftBack.direction = driveEncoderDirection(DriveConfig.BL_REVERSED)
            rightBack.direction = driveEncoderDirection(DriveConfig.BR_REVERSED)
            rightFront.direction = driveEncoderDirection(DriveConfig.FR_REVERSED)
        }

        private var lastLeftFrontPos = 0
        private var lastLeftBackPos = 0
        private var lastRightBackPos = 0
        private var lastRightFrontPos = 0
        private var lastHeading = Rotation2d.exp(0.0)
        /** Wall-clock seconds of the last [update] call, for measured-dt integration. */
        private var lastUpdateSeconds: Double = -1.0
        private var initialized = false
        private var pose = initialPose

        override fun setPose(pose: Pose2d) {
            this.pose = pose
        }

        override fun getPose(): Pose2d = pose

        override fun update(): PoseVelocity2d? {
            val leftFrontPosVel = leftFront.getPositionAndVelocity()
            val leftBackPosVel = leftBack.getPositionAndVelocity()
            val rightBackPosVel = rightBack.getPositionAndVelocity()
            val rightFrontPosVel = rightFront.getPositionAndVelocity()

            var heading: Rotation2d
            var headingDelta: Double

            if (imuAvailable && imu != null) {
                val angles: YawPitchRollAngles = imu.robotYawPitchRollAngles
                heading = Rotation2d.exp(angles.getYaw(AngleUnit.RADIANS))

                FlightRecorder.write(
                    "MECANUM_LOCALIZER_INPUTS",
                    MecanumLocalizerInputsMessage(
                        leftFrontPosVel, leftBackPosVel, rightBackPosVel, rightFrontPosVel, angles,
                    ),
                )

                if (!initialized) {
                    initialized = true
                    lastLeftFrontPos = leftFrontPosVel.position
                    lastLeftBackPos = leftBackPosVel.position
                    lastRightBackPos = rightBackPosVel.position
                    lastRightFrontPos = rightFrontPosVel.position
                    lastHeading = heading
                    return PoseVelocity2d(Vector2d(0.0, 0.0), 0.0)
                }
                headingDelta = heading.minus(lastHeading)
                lastHeading = heading
            } else {
                // DEGRADED PATH: integrate heading from mecanum kinematics omega.
                // Use MEASURED elapsed time (not a fixed 1/50 s) so the integrated
                // heading stays correct regardless of loop rate.
                heading = lastHeading
                val nowSec = now()
                val dt = if (lastUpdateSeconds < 0.0) 0.0 else nowSec - lastUpdateSeconds
                lastUpdateSeconds = nowSec

                val kinematics = MecanumKinematics(
                    PARAMS.inPerTick, PARAMS.lateralInPerTick, PARAMS.trackWidthTicks
                )
                // Velocity-only computation: zero position increment, derivative =
                // measured wheel speed. (DualNum(v, v) made the derivative carry a
                // bogus position component and zeroed out the reported angular rate.)
                val wheelIncrements = MecanumKinematics.WheelIncrements(
                    DualNum<Time>(doubleArrayOf(0.0, (leftFrontPosVel.velocity ?: 0).toDouble())).times(PARAMS.inPerTick),
                    DualNum<Time>(doubleArrayOf(0.0, (leftBackPosVel.velocity ?: 0).toDouble())).times(PARAMS.inPerTick),
                    DualNum<Time>(doubleArrayOf(0.0, (rightBackPosVel.velocity ?: 0).toDouble())).times(PARAMS.inPerTick),
                    DualNum<Time>(doubleArrayOf(0.0, (rightFrontPosVel.velocity ?: 0).toDouble())).times(PARAMS.inPerTick),
                )
                val twist = kinematics.forward(wheelIncrements)
                val vel: PoseVelocity2d = twist.velocity().value()
                val omega: Double = vel.component2()

                FlightRecorder.write(
                    "MECANUM_LOCALIZER_INPUTS_DEGRADED",
                    MecanumLocalizerInputsMessage(leftFrontPosVel, leftBackPosVel, rightBackPosVel, rightFrontPosVel, null),
                )

                if (!initialized) {
                    initialized = true
                    lastLeftFrontPos = leftFrontPosVel.position
                    lastLeftBackPos = leftBackPosVel.position
                    lastRightBackPos = rightBackPosVel.position
                    lastRightFrontPos = rightFrontPosVel.position
                    return PoseVelocity2d(Vector2d(0.0, 0.0), 0.0)
                }
                // Guard against invalid / non-positive dt (clock going backwards,
                // two calls on the same tick): do not integrate heading at all.
                headingDelta = if (dt > 0.0) omega * dt else 0.0
            }

            val twist: Twist2dDual<Time> = kinematics.forward(
                MecanumKinematics.WheelIncrements(
                    DualNum<Time>(
                        doubleArrayOf(
                            (leftFrontPosVel.position - lastLeftFrontPos).toDouble(),
                            (leftFrontPosVel.velocity ?: 0).toDouble(),
                        ),
                    ).times(PARAMS.inPerTick),
                    DualNum<Time>(
                        doubleArrayOf(
                            (leftBackPosVel.position - lastLeftBackPos).toDouble(),
                            (leftBackPosVel.velocity ?: 0).toDouble(),
                        ),
                    ).times(PARAMS.inPerTick),
                    DualNum<Time>(
                        doubleArrayOf(
                            (rightBackPosVel.position - lastRightBackPos).toDouble(),
                            (rightBackPosVel.velocity ?: 0).toDouble(),
                        ),
                    ).times(PARAMS.inPerTick),
                    DualNum<Time>(
                        doubleArrayOf(
                            (rightFrontPosVel.position - lastRightFrontPos).toDouble(),
                            (rightFrontPosVel.velocity ?: 0).toDouble(),
                        ),
                    ).times(PARAMS.inPerTick),
                ),
            )

            lastLeftFrontPos = leftFrontPosVel.position
            lastLeftBackPos = leftBackPosVel.position
            lastRightBackPos = rightBackPosVel.position
            lastRightFrontPos = rightFrontPosVel.position

            lastHeading = heading

            pose = pose.plus(Twist2d(twist.line.value(), headingDelta))

            return twist.velocity().value()
        }
    }

    /** True when localization is running on the odometry pods (vs drive motors). */
    var usingDeadWheels: Boolean = false
        private set

    /**
     * 3-dead-wheel localizer. Two parallel wheels + one perpendicular.
     * If IMU is available, uses it for heading (like classic RR).
     * If IMU is MISSING, integrates heading from dead-wheel velocity omega
     * (computed from the 3-wheel kinematics). Works in degraded mode.
     */
    inner class DeadWheelLocalizer(
        initialPose: Pose2d,
        par0Motor: DcMotorEx,
        par1Motor: DcMotorEx,
        perpMotor: DcMotorEx,
        private val imuAvailable: Boolean,
    ) : Localizer {
        private val par0 = OverflowEncoder(RawEncoder(par0Motor))
        private val par1 = OverflowEncoder(RawEncoder(par1Motor))
        private val perp = OverflowEncoder(RawEncoder(perpMotor))
        private val imu: IMU? = if (imuAvailable) this@MecanumDrive.imu else null

        private val inPerTick = DrivetrainSpec.deadWheelInPerTick
        private val wheels = DrivetrainSpec.deadWheelsIn()
        private val reversed = DrivetrainSpec.deadWheelReversed()

        private var lastPos = IntArray(3)
        private var lastHeading = Rotation2d.exp(0.0)
        /** Wall-clock seconds of the last [update] call, for measured-dt integration. */
        private var lastUpdateSeconds: Double = -1.0
        private var initialized = false
        private var pose = initialPose

        override fun setPose(pose: Pose2d) {
            this.pose = pose
        }

        override fun getPose(): Pose2d = pose

        override fun update(): PoseVelocity2d? {
            val pv = arrayOf(
                par0.getPositionAndVelocity(),
                par1.getPositionAndVelocity(),
                perp.getPositionAndVelocity(),
            )

            // Velocity from 3-wheel kinematics (works with or without IMU)
            val measuredVel = List(3) { i ->
                DeadWheelMath.signed((pv[i].velocity ?: 0).toDouble() * inPerTick, reversed[i])
            }
            val (vx, vy, omega) = DeadWheelMath.velocity(wheels, measuredVel)

            var heading: Rotation2d
            var headingDelta: Double

            if (imuAvailable && imu != null) {
                // IMU path: use IMU for heading
                val angles: YawPitchRollAngles = imu.robotYawPitchRollAngles
                heading = Rotation2d.exp(angles.getYaw(AngleUnit.RADIANS))

                FlightRecorder.write(
                    "DEADWHEEL_LOCALIZER_INPUTS",
                    DeadWheelLocalizerInputsMessage(pv[0], pv[1], pv[2], angles),
                )

                if (!initialized) {
                    initialized = true
                    for (i in 0..2) lastPos[i] = pv[i].position
                    lastHeading = heading
                    return PoseVelocity2d(Vector2d(0.0, 0.0), 0.0)
                }
                headingDelta = heading.minus(lastHeading)
                lastHeading = heading
            } else {
                // DEGRADED PATH: integrate heading from dead-wheel omega using
                // MEASURED elapsed time (not a fixed 1/50 s).
                heading = lastHeading
                val nowSec = now()
                val dt = if (lastUpdateSeconds < 0.0) 0.0 else nowSec - lastUpdateSeconds
                lastUpdateSeconds = nowSec

                FlightRecorder.write(
                    "DEADWHEEL_LOCALIZER_INPUTS_DEGRADED",
                    DeadWheelLocalizerInputsMessage(pv[0], pv[1], pv[2], null),
                )

                if (!initialized) {
                    initialized = true
                    for (i in 0..2) lastPos[i] = pv[i].position
                    return PoseVelocity2d(Vector2d(0.0, 0.0), 0.0)
                }
                // Guard against invalid / non-positive dt.
                headingDelta = if (dt > 0.0) omega * dt else 0.0
            }

            // position increment: least squares for (dx, dy) with heading delta
            val measured = List(3) { i ->
                DeadWheelMath.signed((pv[i].position - lastPos[i]) * inPerTick, reversed[i])
            }
            val (dx, dy) = DeadWheelMath.positionDelta(wheels, measured, headingDelta)

            for (i in 0..2) lastPos[i] = pv[i].position
            lastHeading = heading

            pose = pose.plus(Twist2d(Vector2d(dx, dy), headingDelta))
            return PoseVelocity2d(Vector2d(vx, vy), omega)
        }
    }

    init {
        throwIfModulesAreOutdated(hardwareMap)

        for (module in hardwareMap.getAll(LynxModule::class.java)) {
            module.bulkCachingMode = LynxModule.BulkCachingMode.AUTO
        }

        // Motor names come from the existing hardware configuration (globals.kt).
        leftFront = hardwareMap.get(DcMotorEx::class.java, DrivetrainFrontLeft)
        leftBack = hardwareMap.get(DcMotorEx::class.java, DrivetrainBackLeft)
        rightBack = hardwareMap.get(DcMotorEx::class.java, DrivetrainBackRight)
        rightFront = hardwareMap.get(DcMotorEx::class.java, DrivetrainFrontRight)

        leftFront.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        leftBack.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        rightBack.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        rightFront.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE

        // Motor polarity comes from DriveConfig — the SAME table the NextFTC
        // Drivetrain (teleop / bench OpModes) reads. Before this, the two
        // drive paths had separate direction logic and could disagree about
        // which way the robot actually faced.
        //
        // Road Runner needs the ENCODER direction flipped to match each
        // reversed motor (its RawEncoder reads the raw count, so the motor
        // reversal and the encoder reversal must be applied together) —
        // see the DriveLocalizer below.
        leftFront.direction =
            if (DriveConfig.FL_REVERSED) DcMotorSimple.Direction.REVERSE else DcMotorSimple.Direction.FORWARD
        leftBack.direction =
            if (DriveConfig.BL_REVERSED) DcMotorSimple.Direction.REVERSE else DcMotorSimple.Direction.FORWARD
        rightBack.direction =
            if (DriveConfig.BR_REVERSED) DcMotorSimple.Direction.REVERSE else DcMotorSimple.Direction.FORWARD
        rightFront.direction =
            if (DriveConfig.FR_REVERSED) DcMotorSimple.Direction.REVERSE else DcMotorSimple.Direction.FORWARD

        // IMU is OPTIONAL — but "optional" has to mean it can never take the
        // robot down. Try to initialize it; on any failure, record why and
        // carry on with encoder-only heading.
        var imuDevice: IMU? = null
        var imuError: String? = null
        try {
            val dev = hardwareMap.get(IMU::class.java, RR_IMU_NAME)
            dev.initialize(
                IMU.Parameters(
                    RevHubOrientationOnRobot(PARAMS.logoFacingDirection, PARAMS.usbFacingDirection)
                )
            )
            // initialize() can succeed while the sensor is still settling, so
            // wait for a real quaternion before trusting it.
            val deadline = System.currentTimeMillis() + IMU_INIT_TIMEOUT_MS
            while (System.currentTimeMillis() < deadline) {
                if (dev.robotOrientationAsQuaternion.acquisitionTime != 0L) break
                Thread.sleep(10)
            }
            imuDevice = dev
        } catch (e: Exception) {
            imuError = e.message ?: e.javaClass.simpleName
        }
        imu = imuDevice
        imuProblem = imuError
        imuDevice?.let {
            // Zero the heading so it starts at 0 for the robot's current
            // orientation (see resetHeading).
            try {
                it.resetYaw()
            } catch (_: Exception) {
            }
        }

        voltageSensor = hardwareMap.voltageSensor.iterator().next()

        // Localization: 3 dead wheels when the pods are configured, mecanum
        // drive-motor odometry otherwise (graceful fallback — the robot runs
        // before the pods are bolted on, just with slip-prone localization).
        val par0Motor = tryGetMotor(hardwareMap, DEAD_WHEEL_PAR0_NAME)
        val par1Motor = tryGetMotor(hardwareMap, DEAD_WHEEL_PAR1_NAME)
        val perpMotor = tryGetMotor(hardwareMap, DEAD_WHEEL_PERP_NAME)
        usingDeadWheels = par0Motor != null && par1Motor != null && perpMotor != null
        val imuAvailable = imu != null
        localizer = if (usingDeadWheels) {
            DeadWheelLocalizer(initialPose, par0Motor!!, par1Motor!!, perpMotor!!, imuAvailable)
        } else {
            DriveLocalizer(initialPose, imuAvailable)
        }

        FlightRecorder.write("MECANUM_PARAMS", PARAMS)
    }

    private fun tryGetMotor(hardwareMap: HardwareMap, name: String): DcMotorEx? =
        try {
            hardwareMap.get(DcMotorEx::class.java, name)
        } catch (_: Exception) {
            null
        }

    var pose: Pose2d
        get() = localizer.getPose()
        set(value) = localizer.setPose(value)

    /**
     * Updates the pose estimate. This is the *only* integration point for visual
     * corrections: the [PoseCorrector] (if attached and enabled) is consulted
     * here, and its (already clamped and confidence-gated) corrected pose is
     * written back into the localizer. Road Runner pose estimation is never
     * bypassed — it is merely nudged.
     */
    fun updatePoseEstimate(): PoseVelocity2d? {
        val vel = localizer.update()

        val corrector = poseCorrector
        if (corrector != null && visionCorrectionEnabled) {
            val corrected = corrector.correct(localizer.getPose(), vel, System.nanoTime())
            localizer.setPose(corrected)
        }

        poseHistory.add(localizer.getPose())
        while (poseHistory.size > 100) {
            poseHistory.removeFirst()
        }

        estimatedPoseWriter.write(PoseMessage(localizer.getPose()))

        lastMeasuredVelocity = vel
        updatePushState()
        return vel
    }

    fun setDrivePowers(powers: PoseVelocity2d) {
        lastCommandedVelocity = powers
        val wheelVels = MecanumKinematics(1.0).inverse(
            PoseVelocity2dDual.constant<Time>(powers, 1),
        )

        // Normalize by MAGNITUDE (the quickstart's Math.abs): without abs,
        // negative wheel powers beyond -1 never scale the others down, and the
        // SDK's clip destroys the wheel ratios for mixed-sign commands.
        var maxPowerMag = 1.0
        for (power in wheelVels.all()) {
            maxPowerMag = max(maxPowerMag, abs(power.value()))
        }

        leftFront.power = wheelVels.leftFront[0] / maxPowerMag
        leftBack.power = wheelVels.leftBack[0] / maxPowerMag
        rightBack.power = wheelVels.rightBack[0] / maxPowerMag
        rightFront.power = wheelVels.rightFront[0] / maxPowerMag
    }

    private fun drawPoseHistory(c: com.acmerobotics.dashboard.canvas.Canvas) {
        val xPoints = DoubleArray(poseHistory.size)
        val yPoints = DoubleArray(poseHistory.size)

        var i = 0
        for (t in poseHistory) {
            xPoints[i] = t.position.x
            yPoints[i] = t.position.y
            i++
        }

        c.setStrokeWidth(1)
        c.strokePolyline(xPoints, yPoints)
    }

    inner class FollowTrajectoryAction(val timeTrajectory: TimeTrajectory) : Action {
        private var beginTs = -1.0

        private val xPoints: DoubleArray
        private val yPoints: DoubleArray

    init {
            val disps = range(
                0.0,
                timeTrajectory.path.length(),
                maxOf(2, ceil(timeTrajectory.path.length() / 2).toInt()),
            )
            xPoints = DoubleArray(disps.size)
            yPoints = DoubleArray(disps.size)
            for (i in disps.indices) {
                val p = timeTrajectory.path[disps[i], 1].value()
                xPoints[i] = p.position.x
                yPoints[i] = p.position.y
            }
        }

        override fun run(p: TelemetryPacket): Boolean {
            val t: Double = if (beginTs < 0) {
                beginTs = now()
                0.0
            } else {
                now() - beginTs
            }

            if (t >= timeTrajectory.duration) {
                leftFront.power = 0.0
                leftBack.power = 0.0
                rightBack.power = 0.0
                rightFront.power = 0.0

                return false
            }

            val txWorldTarget = timeTrajectory[t]
            targetPoseWriter.write(PoseMessage(txWorldTarget.value()))

            val robotVelRobot = updatePoseEstimate() ?: PoseVelocity2d(Vector2d(0.0, 0.0), 0.0)

            val command = HolonomicController(
                PARAMS.axialGain, PARAMS.lateralGain, PARAMS.headingGain,
                PARAMS.axialVelGain, PARAMS.lateralVelGain, PARAMS.headingVelGain,
            ).compute(txWorldTarget, localizer.getPose(), robotVelRobot)
            driveCommandWriter.write(DriveCommandMessage(command))

            val wheelVels = kinematics.inverse(command)
            val voltage = voltageSensor.voltage
            val feedforward = MotorFeedforward(
                PARAMS.kS,
                PARAMS.kV / PARAMS.inPerTick,
                PARAMS.kA / PARAMS.inPerTick,
            )
            val leftFrontPower = feedforward.compute(wheelVels.leftFront) / voltage
            val leftBackPower = feedforward.compute(wheelVels.leftBack) / voltage
            val rightBackPower = feedforward.compute(wheelVels.rightBack) / voltage
            val rightFrontPower = feedforward.compute(wheelVels.rightFront) / voltage
            mecanumCommandWriter.write(
                MecanumCommandMessage(voltage, leftFrontPower, leftBackPower, rightBackPower, rightFrontPower),
            )

            leftFront.power = leftFrontPower
            leftBack.power = leftBackPower
            rightBack.power = rightBackPower
            rightFront.power = rightFrontPower

            p.put("x", localizer.getPose().position.x)
            p.put("y", localizer.getPose().position.y)
            p.put("heading (deg)", Math.toDegrees(localizer.getPose().heading.toDouble()))

            val error = txWorldTarget.value().minusExp(localizer.getPose())
            p.put("xError", error.position.x)
            p.put("yError", error.position.y)
            p.put("headingError (deg)", Math.toDegrees(error.heading.toDouble()))

            // only draw when active; only one drive action should be active at a time
            val c = p.fieldOverlay()
            drawPoseHistory(c)

            c.setStroke("#4CAF50")
            Drawing.drawRobot(c, txWorldTarget.value())

            c.setStroke("#3F51B5")
            Drawing.drawRobot(c, localizer.getPose())

            c.setStroke("#4CAF50FF")
            c.setStrokeWidth(1)
            c.strokePolyline(xPoints, yPoints)

            return true
        }

        override fun preview(fieldOverlay: com.acmerobotics.dashboard.canvas.Canvas) {
            fieldOverlay.setStroke("#4CAF507A")
            fieldOverlay.setStrokeWidth(1)
            fieldOverlay.strokePolyline(xPoints, yPoints)
        }
    }

    inner class TurnAction(private val turn: TimeTurn) : Action {
        private var beginTs = -1.0

        override fun run(p: TelemetryPacket): Boolean {
            val t: Double = if (beginTs < 0) {
                beginTs = now()
                0.0
            } else {
                now() - beginTs
            }

            if (t >= turn.duration) {
                leftFront.power = 0.0
                leftBack.power = 0.0
                rightBack.power = 0.0
                rightFront.power = 0.0

                return false
            }

            val txWorldTarget = turn[t]
            targetPoseWriter.write(PoseMessage(txWorldTarget.value()))

            val robotVelRobot = updatePoseEstimate() ?: PoseVelocity2d(Vector2d(0.0, 0.0), 0.0)

            val command = HolonomicController(
                PARAMS.axialGain, PARAMS.lateralGain, PARAMS.headingGain,
                PARAMS.axialVelGain, PARAMS.lateralVelGain, PARAMS.headingVelGain,
            ).compute(txWorldTarget, localizer.getPose(), robotVelRobot)
            driveCommandWriter.write(DriveCommandMessage(command))

            val wheelVels = kinematics.inverse(command)
            val voltage = voltageSensor.voltage
            val feedforward = MotorFeedforward(
                PARAMS.kS,
                PARAMS.kV / PARAMS.inPerTick,
                PARAMS.kA / PARAMS.inPerTick,
            )
            val leftFrontPower = feedforward.compute(wheelVels.leftFront) / voltage
            val leftBackPower = feedforward.compute(wheelVels.leftBack) / voltage
            val rightBackPower = feedforward.compute(wheelVels.rightBack) / voltage
            val rightFrontPower = feedforward.compute(wheelVels.rightFront) / voltage
            mecanumCommandWriter.write(
                MecanumCommandMessage(voltage, leftFrontPower, leftBackPower, rightBackPower, rightFrontPower),
            )

            leftFront.power = leftFrontPower
            leftBack.power = leftBackPower
            rightBack.power = rightBackPower
            rightFront.power = rightFrontPower

            val c = p.fieldOverlay()
            drawPoseHistory(c)

            c.setStroke("#4CAF50")
            Drawing.drawRobot(c, txWorldTarget.value())

            c.setStroke("#3F51B5")
            Drawing.drawRobot(c, localizer.getPose())

            c.setStroke("#7C4DFFFF")
            c.fillCircle(turn.beginPose.position.x, turn.beginPose.position.y, 2.0)

            return true
        }

        override fun preview(fieldOverlay: com.acmerobotics.dashboard.canvas.Canvas) {
            fieldOverlay.setStroke("#7C4DFF7A")
            fieldOverlay.fillCircle(turn.beginPose.position.x, turn.beginPose.position.y, 2.0)
        }
    }

    fun actionBuilder(beginPose: Pose2d): TrajectoryActionBuilder = TrajectoryActionBuilder(
        { turn: TimeTurn -> TurnAction(turn) },
        { t: TimeTrajectory -> FollowTrajectoryAction(t) },
        TrajectoryBuilderParams(
            1e-6,
            ProfileParams(0.25, 0.1, 1e-2),
        ),
        beginPose, 0.0,
        defaultTurnConstraints,
        defaultVelConstraint, defaultAccelConstraint,
    )

    companion object {
        @JvmStatic
        val PARAMS = Params()

        /** How long to wait for the IMU to produce a valid orientation. */
        const val IMU_INIT_TIMEOUT_MS = 1500L

        /**
         * The encoder direction that makes Road Runner's odometry agree with
         * the SDK-adjusted position the NextFTC drivetrain uses.
         *
         * The SDK negates `currentPosition` for a REVERSE motor, and Road
         * Runner's `RawEncoder` negates it once more. So for a motor whose
         * [DriveConfig] flag says REVERSED, the encoder direction must be
         * REVERSE to cancel the extra flip — leaving all four wheels counting
         * UP when the robot drives forward, which is what Road Runner's
         * kinematics assumes.
         */
        @JvmStatic
        fun driveEncoderDirection(motorReversed: Boolean): DcMotorSimple.Direction =
            if (motorReversed) DcMotorSimple.Direction.REVERSE else DcMotorSimple.Direction.FORWARD

        /**
         * Converts a TeleOp driver command (normalized [-1, 1] sticks) into a Road
         * Runner [PoseVelocity2d] in the SAME physical units as [lastMeasuredVelocity]
         * (inches/sec and radians/sec), so push detection can compare apples to apples.
         *
         * Conventions:
         *   * +forward   -> +x (forward)
         *   * +strafe    (teleop + = RIGHT) -> -y, because RR +y is LEFT
         *   * +turn      (teleop + = CLOCKWISE) -> -angular, because RR +angular is CCW
         *   * full-scale forward = [DrivetrainSpec.driveFreeSpeedInPerSec] inches/sec,
         *     derived from [DrivetrainSpec.DRIVE_MOTOR_MAX_RPM] and the wheel diameter.
         *   * full-scale turn    = v_free / turnRadius radians/sec.
         *
         * Never compare normalized stick commands directly with physical velocities.
         */
        @JvmStatic
        fun teleopCommandToVelocity(forward: Double, strafe: Double, turn: Double): PoseVelocity2d {
            val maxSpeed = DrivetrainSpec.driveFreeSpeedInPerSec
            val turnRadiusIn = DrivetrainSpec.turnRadiusCm() / 2.54
            val maxOmega = if (turnRadiusIn > 0.0) maxSpeed / turnRadiusIn else 0.0
            return PoseVelocity2d(
                Vector2d(forward * maxSpeed, -strafe * maxSpeed),
                -turn * maxOmega,
            )
        }
    }
}





