package org.firstinspires.ftc.teamcode

import com.acmerobotics.roadrunner.Pose2d
import com.acmerobotics.roadrunner.PoseVelocity2d
import com.acmerobotics.roadrunner.Vector2d
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import dev.nextftc.hardware.actuators.NextMotor
import org.firstinspires.ftc.teamcode.vision.VisionSubsystem
import org.firstinspires.ftc.teamcode.ControlFrame
import org.firstinspires.ftc.teamcode.vision.VisionTuning
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor
import kotlin.math.PI
import kotlin.math.atan2
import kotlin.math.cos
import kotlin.math.sin

/**
 * ============ BIOBUZZ SELF-PLAY TELEOP — the robot drives itself ============
 *
 * The full superhuman stack runs the driver-controlled period end to end:
 * WorldModel (field memory) + MCTSPlanner (Monte Carlo lookahead + endgame) +
 * distance-based shooter + reflexes. The HUMAN is demoted to supervisor:
 *
 *   * HOLD LEFT TRIGGER = the deadman. Release it and the robot stops.
 *     Everything (motors, intake, flywheel) zeroes within one loop.
 *   * D-pad LEFT/RIGHT picks the alliance during INIT, same as normal teleop.
 *   * Y = force a score run now (soft hint the planner treats as GO_SCORE).
 *   * The whole match is recorded to the Black Box for post-match review.
 *
 * LEGALITY POSTURE: the robot is still 100% controlled through the Driver
 * Station, nothing is transmitted, and a human hand is on the deadman the
 * entire time. See docs/LEGALITY-REVIEW.md before using in official play —
 * and note a referee/inspector may ask; be ready to explain.
 */
@TeleOp(name = "BioBuzz SelfPlay TeleOp", group = "TeleOp")
class BioBuzzSelfPlayTeleOp : LinearOpMode() {

    private lateinit var drive: org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive
    private lateinit var vision: VisionSubsystem
    private lateinit var hive: HiveScoring
    private lateinit var ballLocator: BallLocator
    private lateinit var intake: Intake
    private lateinit var flywheel: FlyWheel
    private val world = WorldModel()
    private val planner = MCTSPlanner(playouts = 32, maxDepth = 3) // fewer playouts for 2Hz teleop
    private val shadowDashboard = ShadowPlannerDashboard()
    private var jamGuard: IntakeJamGuard? = null
    private var tiltGuard: TiltGuard? = null

    // ---- match state ----
    private var ballsHeld = 0
    private var closeSinceNanos = 0L

    // loader fire cycle (behavior-based: advances one tick at a time)
    private enum class FireState { IDLE, STROKING, WATCHING, RECOVERING }
    private var fireState = FireState.IDLE
    private lateinit var loader: Loader
    private val shotDetector by lazy { ShotDetector(flywheel) }

    /** Flywheel RPM last commanded in GO_SCORE — the RECOVERING gate reuses it. */
    private var commandedRpm = 0.0
    /** nanoTime when RECOVERING began (RECOVERING must never deadlock). */
    private var recoverySinceNanos = 0L
    /** Planner target id already pulled this flower approach (one-shot latch). */
    private var flowerPulledId = -1
    private var clockStartRuntime = -1.0
    private var lastDecision: PlannerDecision? = null
    private var lastLoggedDecision: PlannerDecision? = null
    private var lastDecisionNanos = 0L
    private var lastStateLogNanos = 0L

    private fun toCm(inches: Double) = inches * 2.54

    private fun batteryVolts(): Double = ShotPredictor.reportHardwareMapVolts(hardwareMap)

    /** Wraps an angle to (-PI, PI]. */
    private fun wrap(a: Double): Double {
        var x = a
        while (x > PI) x -= 2 * PI
        while (x < -PI) x += 2 * PI
        return x
    }

    /** Bounded (forward, turn) powers steering toward a FIELD-frame goal (cm). */
    private fun steerTo(fx: Double, fy: Double, px: Double, py: Double, h: Double,
                        maxPower: Double = 0.6): PoseVelocity2d {
        val dx = fx - px
        val dy = fy - py
        val desiredHeading = atan2(dy, dx)
        val headingErr = wrap(desiredHeading - h)
        val turn = (0.04 * headingErr).coerceIn(-0.35, 0.35)
        // Body-frame direction to the target is (cos e, sin e) in Road Runner
        // (+y = LEFT, heading CCW): target left of heading (e > 0) needs
        // POSITIVE strafe. The old -sin() crabs away from the target.
        val forward = (0.012 * kotlin.math.hypot(dx, dy) * cos(headingErr))
            .coerceIn(-maxPower, maxPower)
        val strafe = (0.012 * kotlin.math.hypot(dx, dy) * sin(headingErr) * 0.5)
            .coerceIn(-maxPower / 2, maxPower / 2)
        return PoseVelocity2d(Vector2d(forward, strafe), turn)
    }

    override fun runOpMode() {
        // ---- boot-time hardware probe: no full autonomy kit -> stay parked ----
        val hw = HardwareProbe.probe(hardwareMap)
        if (!hw.fullAutonomy) {
            BlackBox.start(hardwareMap.appContext, "selfplay")
            hw.logToBlackBox()
            BlackBox.event("autonomy_unavailable")
            while (!isStopRequested) {
                telemetry.addLine("!! AUTONOMOUS HARDWARE NOT DETECTED !!")
                telemetry.addLine("SelfPlay stays PARKED — run BioBuzz TeleOp (manual) instead.")
                telemetry.addData("Missing", hw.missingList.joinToString(", "))
                telemetry.update()
                sleep(50) // don't busy-spin a core during INIT
            }
            BlackBox.stop()
            return
        }

        drive = org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive(hardwareMap, Pose2d(0.0, 0.0, 0.0))

        val aprilTagProcessor = AprilTagProcessor.Builder()
            .setDrawTagOutline(true)
            .setDrawAxes(false)
            .build()
        val tagOverlay = TagInfoOverlay()
        ballLocator = BallLocator()
        val upFrames = FrameGrabProcessor("UP")
        val frontFrames = FrameGrabProcessor("FRONT")

        vision = VisionSubsystem(
            hardwareMap,
            extraUpProcessors = listOf(aprilTagProcessor, tagOverlay, upFrames),
            extraFrontProcessors = ballLocator.processors + frontFrames +
                DetectorProcessor(hardwareMap.appContext),
        )
        hive = HiveScoring(aprilTagProcessor, vision.aprilTagCameraId)
        intake = Intake()
        flywheel = FlyWheel()
        loader = Loader().also { it.home() }
        jamGuard = IntakeJamGuard(intake)
        tiltGuard = TiltGuard(hardwareMap)

        vision.attachTo(drive, enableCorrections = VisionTuning.FUSION_ENABLED_DEFAULT)
        BlackBox.start(hardwareMap.appContext, "selfplay")
        hw.logToBlackBox()
        ShotPredictor.ensureLoaded(hardwareMap.appContext)

        // ---- INIT: alliance + instructions ----
        while (!isStarted && !isStopRequested) {
            if (gamepad1.dpad_left) SELECTED_TEAM = TeamColor.RED
            else if (gamepad1.dpad_right) SELECTED_TEAM = TeamColor.BLUE
            telemetry.addLine("SELF-PLAY: the robot drives itself.")
            telemetry.addLine("HOLD LEFT TRIGGER = deadman (release = STOP)")
            telemetry.addLine("Y during the match = force a score run")
            telemetry.addData("ALLIANCE", SELECTED_TEAM.name)
            telemetry.addLine("D-pad LEFT/RIGHT selects alliance")
            telemetry.addData("Planner", world.summary())
            telemetry.update()
        }

        clockStartRuntime = runtime
        BlackBox.event("match_start", "alliance" to SELECTED_TEAM.name, "batteryV" to batteryVolts())

        try {
            while (opModeIsActive()) {
                ControlFrame.tick()
                val elapsed = runtime - clockStartRuntime
                val remaining = (MATCH_TELEOP_SECONDS - elapsed).coerceAtLeast(0.0)
                val now = System.nanoTime()

                drive.updatePoseEstimate()
                vision.updateRoadRunnerPose(drive.pose)
                vision.periodic()

                val pose = drive.pose
                val px = toCm(pose.position.x)
                val py = toCm(pose.position.y)
                val h = pose.heading.toDouble()

                val target = hive.update()
                tagOverlay.publish(target)
                world.update(ballLocator.sightings(), px, py, h, now)
                world.updateHive(target, px, py, h, now)

                // ---- planner (2 Hz, logs only on changed decision) ----
                if (now - lastDecisionNanos > 500_000_000L) {
                    var d = planner.decide(world, ballsHeld, remaining, px, py)
                    if (gamepad1.y) {
                        d = PlannerDecision(
                            PlannerAction.GO_SCORE, -1, "Hive",
                            d.targetFieldXcm, d.targetFieldYcm, d.expectedValue, d.estCostSec,
                            "HUMAN HINT (Y): force score",
                        )
                    }
                    if (d.action != lastLoggedDecision?.action ||
                        d.targetId != lastLoggedDecision?.targetId
                    ) {
                        planner.logDecision(d, "selfplay", ballsHeld, remaining)
                        lastLoggedDecision = d
                    }
                    lastDecision = d
                    lastDecisionNanos = now
                }
                // ---- Shadow Planner Dashboard (live FTC Dashboard stream) ----
                shadowDashboard.update(world, ballsHeld, remaining, px, py, lastDecision, planner)
                val decision = lastDecision ?: PlannerDecision(
                    PlannerAction.HOLD, -1, "", 0.0, 0.0, 0.0, 0.0, "starting",
                )

                // ---- deadman + tilt reflex (above everything) ----
                val deadman = gamepad1.left_trigger > 0.5
                val tipping = tiltGuard?.isTipping() ?: false
                if (!deadman || tipping) {
                    drive.setDrivePowers(PoseVelocity2d(Vector2d(0.0, 0.0), 0.0))
                    intake.stop()
                    flywheel.stop()
                    // Return the loader home: a mid-stroke release otherwise
                    // leaves the gate open at STROKE_END, and since strokes
                    // require phase == HOME, firing would be dead for the rest
                    // of the match (nothing else in SelfPlay homes it).
                    if (loader.available) loader.home()
                    shotDetector.idle()
                    fireState = FireState.IDLE
                    BlackBox.event(
                        "guard", "deadman" to deadman, "tipping" to tipping,
                        "tiltDeg" to (tiltGuard?.tiltDeg() ?: 0.0),
                    )
                    telemetry.addData(
                        "SUPERVISOR",
                        if (!deadman) "DEADMAN RELEASED — stopped" else "TILT GUARD — stopped",
                    )
                    telemetry.update()
                    continue
                }

                execute(decision, px, py, h)

                // ---- black box state @ BLACKBOX_STATE_LOG_HZ ----
                if (now - lastStateLogNanos > (1e9 / BLACKBOX_STATE_LOG_HZ).toLong()) {
                    lastStateLogNanos = now
                    BlackBox.event(
                        "state",
                        "remainingSec" to remaining,
                        "poseXcm" to px, "poseYcm" to py, "headingRad" to h,
                        "ballsHeld" to ballsHeld,
                        "action" to decision.action.name, "targetId" to decision.targetId,
                        "flyL" to flywheel.measuredLeftRpm(),
                        "flyR" to flywheel.measuredRightRpm(),
                        "batteryV" to batteryVolts(),
                        "tiltDeg" to (tiltGuard?.tiltDeg() ?: 0.0),
                    )
                    world.logToBlackBox()
                }

                telemetry.addData("CLOCK", "%.0fs left".format(remaining))
                telemetry.addData("PLAN", "%s — %s".format(decision.action.name, decision.rationale))
                telemetry.addData("BALLS HELD", "$ballsHeld / $MAX_BALLS_HELD")
                telemetry.addData("WORLD", world.summary())
                telemetry.addData(
                    "FLYWHEEL", "%.0f / %.0f RPM".format(
                        flywheel.measuredLeftRpm(), flywheel.measuredRightRpm(),
                    ),
                )
                telemetry.update()
            }
        } finally {
            drive.setDrivePowers(PoseVelocity2d(Vector2d(0.0, 0.0), 0.0))
            intake.stop()
            flywheel.stop()
            ControlFrame.forcePoll()
            vision.close()
            BlackBox.stop()
        }
    }


    /**
     * Executes one planner decision for this control-loop tick. The planner
     * re-decides at 2 Hz, so these must all be *behaviors*, not blocking
     * sequences — the robot can change its mind mid-approach.
     */
    private fun execute(d: PlannerDecision, px: Double, py: Double, h: Double) {
        val now = System.nanoTime()
        val hivePos = world.hive()

        // One flower pull per approach: the latch clears as soon as the
        // planner leaves USE_FLOWER (or targets a different flower).
        if (d.action != PlannerAction.USE_FLOWER) flowerPulledId = -1

        when (d.action) {
            PlannerAction.COLLECT_BALL -> {
                // Steer toward the REMEMBERED field position — works even when
                // the ball is out of view (that's the memory advantage).
                drive.setDrivePowers(steerTo(d.targetFieldXcm, d.targetFieldYcm, px, py, h))

                val jam = jamGuard?.update(spinning = true)
                if (jam == IntakeJamGuard.State.JAMMED) {
                    BlackBox.event("jam", "state" to "JAMMED")
                    // give the pickup a rest: stop intake until the guard resets
                }

                // Pickup confirmation: a live ball stuck at the intake for
                // PICKUP_SETTLE_MS counts as collected (same rule as autonomous).
                val live = ballLocator.nearestBall()
                if (live != null && live.distanceCm <= PICKUP_CONFIRM_CM) {
                    if (closeSinceNanos == 0L) {
                        closeSinceNanos = now
                    } else if ((now - closeSinceNanos) / 1e6 >= PICKUP_SETTLE_MS) {
                        ballsHeld = (ballsHeld + 1).coerceAtMost(MAX_BALLS_HELD)
                        closeSinceNanos = 0L
                        BlackBox.event("ball_collected", "label" to live.label, "ballsHeld" to ballsHeld)
                    }
                } else {
                    closeSinceNanos = 0L
                }
            }

            PlannerAction.GO_SCORE -> {
                // The intake must NOT run during a score run: NextFTC velocity
                // setpoints persist until changed, so if the previous action was
                // COLLECT_BALL the intake would keep spinning through firing.
                // update(spinning=false) zeroes it AND resets a JAMMED guard
                // (otherwise one jam would disable the intake all match).
                jamGuard?.update(spinning = false)

                val t = hive.update()
                val rpm = ShotPredictor.adjustRpm(
                    hive.shootingRpmFor(t),
                    t?.distanceCm ?: 0.0,
                    "selfplay",
                )
                flywheel.spinToRpm(rpm)
                commandedRpm = rpm

                if (t != null) {
                    // live tag: servo into the window while already at speed
                    val (f, turn) = hive.rangeCorrection(t)
                    drive.setDrivePowers(PoseVelocity2d(Vector2d(f, 0.0), turn))
                    val aligned = hive.zoneOf(t) == ScoringZone.ALIGNED
                    if (aligned && flywheel.isAtSpeed(rpm) &&
                        fireState == FireState.IDLE && ballsHeld > 0 &&
                        loader.phase == LoaderPhase.HOME && !loader.busy
                    ) {
                        loader.stroke()
                        fireState = FireState.STROKING
                        BlackBox.event(
                            "fire_start", "rpm" to rpm,
                            "distanceCm" to t.distanceCm, "ballsHeld" to ballsHeld,
                        )
                    }
                } else if (hivePos != null) {
                    // remember where the hive was even with no live tag
                    drive.setDrivePowers(steerTo(hivePos.first, hivePos.second, px, py, h))
                } else {
                    drive.setDrivePowers(
                        PoseVelocity2d(Vector2d(0.0, 0.0), -HiveScoringConfig.SEARCH_TURN_POWER),
                    )
                }
            }

            PlannerAction.USE_FLOWER -> {
                val live = ballLocator.nearestFlower()
                if (live != null && live.distanceCm <= FLOWER_APPROACH_CM + 5.0) {
                    // In range: stop, pull ONCE (latched), then keep swallowing.
                    // The pull routine may block briefly — it must stay SHORT,
                    // because the deadman is only polled between ticks.
                    drive.setDrivePowers(PoseVelocity2d(Vector2d(0.0, 0.0), 0.0))
                    if (d.targetId != flowerPulledId) {
                        flowerPulledId = d.targetId
                        flowerPullRoutine(intake, loader)
                        intake.spinUp(1.0) // swallow the released ball (setpoint persists)
                        ballsHeld = (ballsHeld + 1).coerceAtMost(MAX_BALLS_HELD)
                        BlackBox.event(
                            "flower_pull", "distanceCm" to live.distanceCm,
                            "ballsHeld" to ballsHeld,
                        )
                    }
                    // keep the jam watch on while swallowing at the flower
                    jamGuard?.update(spinning = true)
                } else {
                    drive.setDrivePowers(steerTo(d.targetFieldXcm, d.targetFieldYcm, px, py, h))
                    jamGuard?.update(spinning = true)
                }
            }

            PlannerAction.PARK, PlannerAction.HOLD -> {
                drive.setDrivePowers(PoseVelocity2d(Vector2d(0.0, 0.0), 0.0))
                // update(false) idles the intake and resets a jammed guard
                jamGuard?.update(spinning = false)
                intake.stop()
                if (d.action == PlannerAction.PARK) flywheel.stop() else flywheel.ready()
            }
        }

        // ---- loader fire cycle: stroke -> watch RPM dip -> retract -> recover ----
        // Advances under EVERY action so the gate never stays open if the
        // planner switches away from GO_SCORE mid-cycle. Strokes are only
        // STARTED in GO_SCORE; this machine only finishes what was started.
        // (The intake never runs during fire — the loader meters balls.)
        when (fireState) {
            FireState.STROKING ->
                if (!loader.busy) {
                    shotDetector.arm()
                    fireState = FireState.WATCHING
                }
            FireState.WATCHING ->
                when (shotDetector.update()) {
                    ShotDetector.State.CONFIRMED, ShotDetector.State.MISSED -> {
                        ballsHeld = (ballsHeld - 1).coerceAtLeast(0)
                        loader.retract()
                        fireState = FireState.RECOVERING
                        recoverySinceNanos = now
                        BlackBox.event("fire_ball_done", "ballsHeld" to ballsHeld)
                    }
                    else -> {}
                }
            FireState.RECOVERING -> {
                // Wheels re-verified at speed before the next ball — but with a
                // timeout, so a stopped flywheel (e.g. the planner parked) can
                // never wedge the machine in RECOVERING forever.
                val recovered = !loader.busy && loader.phase == LoaderPhase.HOME &&
                    flywheel.isAtSpeed(commandedRpm)
                val timedOut = recoverySinceNanos != 0L &&
                    now - recoverySinceNanos > (SHOT_RECOVERY_TIMEOUT_MS * 1e6).toLong()
                if (recovered || timedOut) {
                    shotDetector.idle()
                    fireState = FireState.IDLE
                    recoverySinceNanos = 0L
                }
            }
            FireState.IDLE -> {}
        }
    }
}