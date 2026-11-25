package org.firstinspires.ftc.teamcode

import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.acmerobotics.roadrunner.Action
import com.acmerobotics.roadrunner.Pose2d
import com.acmerobotics.roadrunner.PoseVelocity2d
import com.acmerobotics.roadrunner.Vector2d
import com.pedropathing.ivy.behaviors.EndCondition
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import dev.nextftc.hardware.actuators.NextMotor
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive
import org.firstinspires.ftc.teamcode.vision.VisionSubsystem
import org.firstinspires.ftc.teamcode.vision.VisionTuning
import org.firstinspires.ftc.teamcode.vision.visionGuidance
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor
import kotlin.math.abs
import kotlin.math.atan2
import kotlin.math.cos
import kotlin.math.hypot
import kotlin.math.sin

/**
 * Field-specific route plan. The collection phase is now vision-directed
 * (see globals.kt priority + collection constants); only the park move
 * remains a fixed trajectory. TUNE for the actual venue.
 */
object AutoPlan {
    const val PARK_BACKOFF_IN = 12.0 // TUNE: back-off after firing
}

/**
 * ============ BIOBUZZ AUTONOMOUS — the closed ecosystem ============
 *
 * Phases (each timeout-bounded, safe-stopped, telemetry-visible):
 *   1. COLLECT: intake on, RR trajectory sweep through the collection area.
 *   2. POLISH:  vision-guidance command nudges the pose against stationary
 *              visual references (no-ops unless vision is confident).
 *   3. ALIGN:   search for the hive tag, servo into the shooting window;
 *              flywheel pre-spins to the RPM the live distance needs.
 *   4. FIRE:    spin to the calibrated RPM, wait until BOTH wheels are at
 *              speed, then feed balls through for FIRE_TIME_MS.
 *   5. PARK:    back off from the hive and stop.
 *
 * Prereqs on the real robot: RR tuning (MecanumDrive.PARAMS), "imu" configured,
 * vision test procedure passed, [HiveScoringConfig] measured.
 *
 * ALLIANCE: pick RED or BLUE on the gamepad during INIT (D-pad LEFT/RIGHT);
 * only the selected alliance's hive tags are ever targeted (SELECTED_TEAM).
 */
@Autonomous(name = "BioBuzz Autonomous", group = "Auto")
class BioBuzzAutonomous : LinearOpMode() {

    private lateinit var drive: MecanumDrive
    private lateinit var vision: VisionSubsystem
    private lateinit var hive: HiveScoring
    private lateinit var tagOverlay: TagInfoOverlay
    private lateinit var ballLocator: BallLocator
    private lateinit var intake: Intake
    private lateinit var flywheel: FlyWheel
    private lateinit var loader: Loader
    private lateinit var jamGuard: IntakeJamGuard
    private lateinit var clock: PhaseClock
    private lateinit var pickupSensor: PickupCurrentSensor
    private val world = WorldModel()

    override fun runOpMode() {
        // ---- boot-time hardware probe: no full autonomy kit -> stay parked ----
        val hw = HardwareProbe.probe(hardwareMap)
        if (!hw.fullAutonomy) {
            BlackBox.start(hardwareMap.appContext, "auto")
            hw.logToBlackBox()
            BlackBox.event("autonomy_unavailable")
            while (!isStopRequested) {
                telemetry.addLine("!! AUTONOMOUS HARDWARE NOT DETECTED !!")
                telemetry.addLine("Autonomous stays PARKED — run a TeleOp instead.")
                telemetry.addData("Missing", hw.missingList.joinToString(", "))
                telemetry.update()
                sleep(50) // don't busy-spin a core during INIT
            }
            BlackBox.stop()
            return
        }

        // ================= initialize =================
        drive = MecanumDrive(hardwareMap, Pose2d(0.0, 0.0, 0.0))

        // Known-target sensors share the localization portals (AprilTag on the
        // UP camera, ball blobs on the FRONT camera — no extra cameras).
        val aprilTagProcessor = AprilTagProcessor.Builder()
            .setDrawTagOutline(true)
            .setDrawAxes(false)
            .setDrawCubeProjection(false)
            .build()
        tagOverlay = TagInfoOverlay()
        ballLocator = BallLocator()

        vision = VisionSubsystem(
            hardwareMap,
            extraUpProcessors = listOf(aprilTagProcessor, tagOverlay, FrameGrabProcessor("UP")),
            extraFrontProcessors = ballLocator.processors + FrameGrabProcessor("FRONT") +
                DetectorProcessor(hardwareMap.appContext),
        )
        hive = HiveScoring(aprilTagProcessor, vision.aprilTagCameraId)
        intake = Intake()
        flywheel = FlyWheel()
        loader = Loader().also { it.home() }
        jamGuard = IntakeJamGuard(intake)

        vision.attachTo(drive, enableCorrections = VisionTuning.FUSION_ENABLED_DEFAULT)
        BlackBox.start(hardwareMap.appContext, "auto")
        hw.logToBlackBox()
        ShotPredictor.ensureLoaded(hardwareMap.appContext)
        // Model provenance: which detector build + authority modes ran today.
        BlackBox.event(
            "nn_model",
            "version" to NN_MODEL_VERSION,
            "detectorMode" to NN_DETECTOR_MODE.name,
            "shotPredictorMode" to SHOT_PREDICTOR_MODE.name,
        )
        NeuralReferee.resetStats()

        // ---- INIT: pick your alliance before the match starts ----
        // D-pad LEFT = RED, D-pad RIGHT = BLUE. HiveScoring only ever targets
        // the selected alliance's hives, so this must match the match sheet.
        while (!isStarted && !isStopRequested) {
            if (gamepad1.dpad_left) SELECTED_TEAM = TeamColor.RED
            else if (gamepad1.dpad_right) SELECTED_TEAM = TeamColor.BLUE

            telemetry.addLine("BioBuzz autonomous: RR + hybrid vision + AprilTag scoring")
            telemetry.addData("Cameras", vision.cameraMode)
            telemetry.addData(
                "Vision corrections",
                if (drive.visionCorrectionEnabled) "ENABLED" else "disabled (pass vision tests first)",
            )
            telemetry.addData(
                "Shoot range",
                "${HiveScoringConfig.SHOOT_MIN_RANGE_CM.toInt()}-" +
                    "${ShooterModel.effectiveMaxRangeCm().toInt()} cm " +
                    "(${ShooterModel.summary()}), " +
                    "|angle| <= ${HiveScoringConfig.SHOOT_MAX_BEARING_DEG.toInt()} deg",
            )
            telemetry.addData("ALLIANCE", SELECTED_TEAM.name)
            telemetry.addLine("Select with D-pad: LEFT = RED, RIGHT = BLUE")
            if (MecanumDrive.PARAMS.kV == 0.0 || MecanumDrive.PARAMS.axialGain == 0.0) {
                telemetry.addLine("!! RR UNTUNED (MecanumDrive.PARAMS are zeros) !!")
                telemetry.addLine("-> trajectory moves (e.g. PHASE 5 park) will NOT drive the robot")
            }
            telemetry.update()
        }

        clock = PhaseClock()
        pickupSensor = PickupCurrentSensor(intake)

        try {
            // ---- THE SCORE LOOP: keep cycling while a full cycle fits ----
            var cycle = 0
            val inventory = HopperInventory() // physical hopper inventory, persists across cycles
            var totalScored = 0
            while (opModeIsActive()) {
                cycle++
                NextMotor.motorEventLoop.poll()
                BlackBox.event("cycle_start", "cycle" to cycle, "remainingSec" to clock.remainingSec())
                telemetry.addData("AUTO CYCLE", cycle)
                telemetry.addData("Clock", "%.1fs left".format(clock.remainingSec()))
                telemetry.update()

                // ---- PHASE 1: COLLECT (planner-driven, memory-backed) ----
                BlackBox.event("phase", "name" to "1-collect")
                val maxToCollect = inventory.remainingCapacity()
                val collected = collectBalls(
                    clock.deadlineNanos(
                        AUTONOMOUS_COLLECT_TIMEOUT_MS,
                        reserveSec = AUTONOMY_PARK_RESERVE_SEC + AUTONOMY_FIRE_RESERVE_SEC,
                    ),
                    maxToCollect,
                    inventory.held, // true held count for the planner
                )
                inventory.addCollected(collected)
                BlackBox.event("collect_done", "balls" to collected, "held" to inventory.held)
                telemetry.addData("PHASE 1 done", "${inventory.held} / $MAX_BALLS_HELD balls")

                // ---- PHASE 1b: FLOWER (optional; needs capacity AND time) ----
                if (FLOWER_PULL_ENABLED && !inventory.isFull() &&
                    clock.fits(MIN_CYCLE_REMAINING_SEC + AUTONOMY_FIRE_RESERVE_SEC)
                ) {
                    telemetry.addLine("PHASE 1b: flower pull")
                    BlackBox.event("phase", "name" to "1b-flower")
                    if (useFlowerIfVisible(
                        clock.deadlineNanos(
                            FLOWER_PULL_TIMEOUT_MS,
                            reserveSec = AUTONOMY_PARK_RESERVE_SEC + AUTONOMY_FIRE_RESERVE_SEC,
                        )
                    )) {
                        inventory.addFlower()
                        BlackBox.event("flower_done", "held" to inventory.held)
                    }
                }

                // ---- PHASE 2: POLISH (first cycle only — one-time cleanup) ----
                if (cycle == 1) {
                    telemetry.addLine("PHASE 2: vision-guidance polish")
                    BlackBox.event("phase", "name" to "2-polish")
                    runGuidance()
                }

                // ---- PHASE 3 + 4: ALIGN + FIRE (only with balls to shoot) ----
                if (!inventory.isEmpty()) {
                    telemetry.addLine("PHASE 3: hive alignment")
                    BlackBox.event("phase", "name" to "3-align")
                    flywheel.spinToRpm(
                        hive.update()?.let { ShooterModel.rpmForDistance(it.distanceCm) }
                            ?: FLYWHEEL_PRIME_RPM
                    )
                    val aligned = alignToHive(
                        clock.deadlineNanos(
                            HiveScoringConfig.ALIGN_TIMEOUT_MS,
                            reserveSec = AUTONOMY_PARK_RESERVE_SEC + AUTONOMY_FIRE_RESERVE_SEC,
                        )
                    )

                    if (aligned) {
                        val result = fireBalls(inventory.held)
                        inventory.eject(result.attempted)
                        inventory.confirm(result.confirmed)
                        totalScored = inventory.totalScored
                        BlackBox.event(
                            "cycle_done",
                            "cycle" to cycle, "attempted" to result.attempted,
                            "confirmed" to result.confirmed, "totalScored" to totalScored,
                            "held" to inventory.held,
                        )
                    } else {
                        // Align failed: physical inventory is PRESERVED for the next cycle.
                        BlackBox.event("fire_skipped", "reason" to "align failed", "held" to inventory.held)
                        telemetry.addData("PHASE 4", "ALIGN FAILED - skipping fire (safe), ${inventory.held} balls held")
                        telemetry.update()
                    }
                    flywheel.stop()
                } else if (!clock.fits(MIN_CYCLE_REMAINING_SEC)) {
                    // Nothing collected and no time for another try: park.
                    break
                }

                if (!clock.fits(MIN_CYCLE_REMAINING_SEC)) break
            }

            // ---- PHASE 5: PARK (time reserved by the clock; always runs) ----
            telemetry.addData("PHASE 5", "park (%.0fs left)".format(clock.remainingSec()))
            telemetry.update()
            BlackBox.event("phase", "name" to "5-park")
            // Back off ALONG THE ROBOT'S OWN HEADING (the old -X world move
            // was sideways/toward-hive whenever align had rotated the bot).
            val hdg = drive.pose.heading.toDouble()
            runAction(
                drive.actionBuilder(drive.pose)
                    .strafeToConstantHeading(
                        Vector2d(
                            drive.pose.position.x - AutoPlan.PARK_BACKOFF_IN * cos(hdg),
                            drive.pose.position.y - AutoPlan.PARK_BACKOFF_IN * sin(hdg),
                        ),
                    )
                    .build(),
            )
        } finally {
            // Always leave the robot safe, no matter where the phases exit.
            drive.setDrivePowers(PoseVelocity2d(Vector2d(0.0, 0.0), 0.0))
            intake.stop()
            flywheel.stop()
            NextMotor.motorEventLoop.poll()
            vision.close()
            BlackBox.event("auto_end")
            BlackBox.stop()
        }
    }

    // ================= phase helper loops =================

    /**
     * Runs a Road Runner action while continuously feeding the hybrid vision
     * fusion + telemetry. Stops the drivetrain immediately if the OpMode ends
     * mid-trajectory (action.run() does not know about opModeIsActive).
     */
    private fun runAction(action: Action) {
        val packet = TelemetryPacket()
        var running = true
        while (running && opModeIsActive()) {
            NextMotor.motorEventLoop.poll()
            running = action.run(packet) // also updates pose (and vision correction)
            vision.updateRoadRunnerPose(drive.pose)
            report()
        }
        if (!opModeIsActive()) {
            drive.setDrivePowers(PoseVelocity2d(Vector2d(0.0, 0.0), 0.0))
        }
    }

    /**
     * Inline-executes the NextFTC vision-guidance command. It only moves the
     * robot while vision confidence is high; the command's own timeout ends it.
     */
    private fun runGuidance() {
        val cmd = visionGuidance(drive, vision)
        cmd.start()
        while (!cmd.done() && opModeIsActive()) {
            NextMotor.motorEventLoop.poll()
            drive.updatePoseEstimate()
            vision.updateRoadRunnerPose(drive.pose)
            cmd.execute()
            report()
        }
        cmd.end(if (cmd.done()) EndCondition.NATURALLY else EndCondition.INTERRUPTED)
    }

    /**
     * PHASE 1 core: collection driven by this branch's own decision stack.
     * Each loop:
     *   1. report() fuses this frame's sightings into the WorldModel,
     *   2. the StrategicPlanner picks the action. COLLECT_BALL commits to a
     *      remembered TRACK — a challenger must win TARGET_SWITCH_CONFIRM_LOOPS
     *      consecutive picks to unseat it, which kills target dithering;
     *      GO_SCORE / USE_FLOWER / PARK exit the phase early so the phase
     *      machine can act on the strategy immediately,
     *   3. dodge opponent robots sitting in the pursuit corridor,
     *   4. pursue the track's FIELD position — dead-reckoned on the fused
     *      pose when the ball has left the FOV — with a turn-toward-ball
     *      term so wide bearings rotate the intake onto the ball,
     *   5. confirm pickup by proximity-settle, then removeBall() so the
     *      map never re-chases a collected ball,
     *   6. nothing worth doing (HOLD): sweep-search in place instead of the
     *      old blind forward creep.
     * Bounded by AUTONOMOUS_COLLECT_TIMEOUT_MS.
     */
    /**
     * PHASE 1 core: collection driven by this branch's own decision stack.
     * Each loop:
     *   1. report() fuses this frame's sightings into the WorldModel,
     *   2. the StrategicPlanner picks the action. COLLECT_BALL commits to a
     *      remembered TRACK — a challenger must win TARGET_SWITCH_CONFIRM_LOOPS
     *      consecutive picks to unseat it, which kills target dithering;
     *      GO_SCORE / USE_FLOWER / PARK exit the phase early so the phase
     *      machine can act on the strategy immediately,
     *   3. dodge opponent robots sitting in the pursuit corridor,
     *   4. pursue the track's FIELD position — dead-reckoned on the fused
     *      pose when the ball has left the FOV — with a turn-toward-ball
     *      term so wide bearings rotate the intake onto the ball,
     *   5. confirm pickup by proximity-settle, then removeBall() so the
     *      map never re-chases a collected ball,
     *   6. nothing worth doing (HOLD): sweep-search in place instead of the
     *      old blind forward creep.
     * Bounded by AUTONOMOUS_COLLECT_TIMEOUT_MS.
     *
     * @param maxToCollect remaining hopper capacity — collection stops here so
     *                     the hopper is never over-filled.
     * @param alreadyHeld  true physical inventory carried over from previous
     *                     cycles; fed to the planner (as alreadyHeld + collected)
     *                     so it reasons about the real held count.
     */
    private fun collectBalls(deadlineNanos: Long, maxToCollect: Int, alreadyHeld: Int): Int {
        var collected = 0
        var committedId: Int? = null
        var challengerId: Int? = null
        var challengerWins = 0
        var closeSinceNanos = 0L
        var dodgeSign = 0        // latched dodge direction (+left / -right in RR strafe terms)
        var dodgeUntilNanos = 0L
        var blockedSinceNanos = 0L
        val planner = StrategicPlanner()

        // The jam guard OWNS the intake during collection (full speed, coughs
        // on jams, brakes hard when fully jammed) — a jam can no longer stall
        // the phase silently.
        while (opModeIsActive() && collected < maxToCollect && System.nanoTime() < deadlineNanos) {
            NextMotor.motorEventLoop.poll()
            drive.updatePoseEstimate()
            vision.updateRoadRunnerPose(drive.pose)
            report() // fuses sightings + hive into the WorldModel (and Black Box)

            val guardState = jamGuard.update(spinning = true)
            if (guardState == IntakeJamGuard.State.JAMMED) {
                // Repeated coughs failed: stop collecting with what we hold.
                BlackBox.event("collect_jam_stopped", "balls" to collected)
                telemetry.addData("COLLECT", "INTAKE JAMMED — ending collection with $collected balls")
                telemetry.update()
                break
            }

            val now = System.nanoTime()
            val px = drive.pose.position.x * 2.54
            val py = drive.pose.position.y * 2.54
            val hdg = drive.pose.heading.toDouble()
            val bumped = pickupSensor.update(spinning = true)
            // The planner reasons in MATCH time, not collect-window time.
            val matchRemainingSec = clock.remainingSec()

            // ---- 2. planner decision ----
            val decision = planner.decide(world, alreadyHeld + collected, matchRemainingSec, px, py)
            when (decision.action) {
                PlannerAction.GO_SCORE, PlannerAction.USE_FLOWER, PlannerAction.PARK -> {
                    // Leave collection to the phase machine (flower / align / fire / park).
                    BlackBox.event(
                        "collect_exit",
                        "action" to decision.action.name,
                        "balls" to collected,
                        "why" to decision.rationale,
                    )
                    drive.setDrivePowers(PoseVelocity2d(Vector2d(0.0, 0.0), 0.0))
                    intake.stop()
                    return collected
                }
                else -> {}
            }

            // ---- commitment with confirmation hysteresis ----
            if (decision.action == PlannerAction.COLLECT_BALL) {
                val current = committedId?.let { world.trackById(it) }
                if (current == null) {
                    committedId = decision.targetId
                    challengerId = null; challengerWins = 0
                    closeSinceNanos = 0L // fresh commitment: no settle time carried over
                } else if (decision.targetId != committedId) {
                    if (challengerId == decision.targetId) challengerWins++ else challengerWins = 1
                    challengerId = decision.targetId
                    if (challengerWins >= TARGET_SWITCH_CONFIRM_LOOPS) {
                        BlackBox.event(
                            "target_switch",
                            "from" to committedId, "to" to decision.targetId, "why" to decision.rationale,
                        )
                        committedId = decision.targetId
                        challengerId = null; challengerWins = 0
                        closeSinceNanos = 0L // settle time cannot transfer between balls
                    }
                } else {
                    challengerId = null; challengerWins = 0
                }
            }

            var target = committedId?.let { world.trackById(it) }
            // Chase eligibility: fresh-enough sighting of a solid-enough blob.
            if (target != null &&
                (target.ageSec(now) > DEAD_RECKON_MAX_AGE_SEC || target.lastAreaPx < CHASE_MIN_CONTOUR_AREA_PX)
            ) {
                committedId = null
                target = null
            }

            if (target == null) {
                // ---- 6. nothing worth chasing: sweep-search in place ----
                closeSinceNanos = 0L
                dodgeSign = 0
                telemetry.addData("COLLECT", "no target — sweep | ${world.summary()}")
                telemetry.update()
                drive.setDrivePowers(PoseVelocity2d(Vector2d(0.0, 0.0), -SEARCH_SWEEP_TURN_POWER))
                continue
            }

            val (distCm, bearingDeg) = target.relativeToRobot(px, py, hdg)

            // ---- 3. opponent dodge: a robot blob (live OR velocity-predicted)
            //      in the pursuit corridor blocks the line ----
            val blocker = ballLocator.visibleRobots().firstOrNull {
                abs(it.bearingDeg - bearingDeg) < AVOID_ROBOT_BEARING_DEG && it.distanceCm < distCm
            } ?: predictedBlocker(px, py, hdg, bearingDeg, distCm, now)
            if (blocker != null) {
                // Latch the dodge direction once; only give the target up
                // (and re-approach from a new angle) after a long block.
                if (dodgeSign == 0) {
                    dodgeSign = if (blocker.bearingDeg >= 0) 1 else -1
                    dodgeUntilNanos = now + (DODGE_LATCH_MS * 1e6).toLong()
                }
                if (blockedSinceNanos == 0L) blockedSinceNanos = now
                closeSinceNanos = 0L

                val givingUp = (now - blockedSinceNanos) / 1e6 >= BLOCKED_GIVEUP_MS
                val stillLatched = now < dodgeUntilNanos
                if (givingUp || !stillLatched) {
                    dodgeSign = 0
                    blockedSinceNanos = 0L
                }
                if (givingUp) {
                    committedId = null // re-plan around the blocker
                    BlackBox.event("dodge_giveup", "targetId" to target.id)
                }
                telemetry.addData(
                    "COLLECT dodge",
                    "robot %.0fcm %+.0fdeg".format(blocker.distanceCm, blocker.bearingDeg),
                )
                telemetry.update()
                drive.setDrivePowers(PoseVelocity2d(Vector2d(0.0, dodgeSign * AVOID_ROBOT_STRAFE), 0.0))
                continue
            } else {
                dodgeSign = 0
                blockedSinceNanos = 0L
            }

            // ---- 4. pursue the track's field position (live or remembered) ----
            val forward = (HiveScoringConfig.CAPTURE_RANGE_GAIN *
                (distCm - HiveScoringConfig.CAPTURE_APPROACH_CM))
                .coerceIn(-HiveScoringConfig.ASSIST_MAX_POWER, HiveScoringConfig.ASSIST_MAX_POWER)
            val strafe = (-HiveScoringConfig.CAPTURE_BEARING_GAIN * bearingDeg)
                .coerceIn(-HiveScoringConfig.ASSIST_MAX_POWER, HiveScoringConfig.ASSIST_MAX_POWER)
            // Turn toward the ball while translating (RR: +turn CCW).
            val turn = (-CAPTURE_TURN_GAIN * bearingDeg)
                .coerceIn(-CAPTURE_TURN_MAX_POWER, CAPTURE_TURN_MAX_POWER)
            drive.setDrivePowers(PoseVelocity2d(Vector2d(forward, strafe), turn))

            val live = target.ageSec(now) <= WM_VISIBLE_FRESH_SEC
            telemetry.addData(
                "COLLECT $collected/$MAX_BALLS_HELD",
                "%s %.0fcm %+.1fdeg %s".format(
                    target.label, distCm, bearingDeg, if (live) "[live]" else "[memory]",
                ),
            )
            telemetry.update()

            // ---- 5. pickup confirmation: current bump (physical) or settle ----
            val bumpPickup = bumped && distCm <= PICKUP_CONFIRM_CM * 1.5
            val settled = if (distCm <= PICKUP_CONFIRM_CM) {
                if (closeSinceNanos == 0L) {
                    closeSinceNanos = now
                    false
                } else {
                    (now - closeSinceNanos) / 1e6 >= PICKUP_SETTLE_MS
                }
            } else {
                closeSinceNanos = 0L
                false
            }

            if (bumpPickup || settled) {
                collected++
                world.removeBall(target.id)
                committedId = null
                closeSinceNanos = 0L
                BlackBox.event(
                    "pickup",
                    "label" to target.label, "how" to if (bumpPickup) "current" else "settled",
                )
                telemetry.addData(
                    "COLLECT pickup",
                    "${target.label} (%s)".format(if (bumpPickup) "current bump" else "settled"),
                )
                telemetry.update()
            }
        }

        drive.setDrivePowers(PoseVelocity2d(Vector2d(0.0, 0.0), 0.0))
        jamGuard.update(spinning = false) // zeroes the intake + resets the guard
        intake.stop()
        return collected
    }

    /**
     * A remembered opponent PREDICTED ([DODGE_LOOKAHEAD_SEC] ahead, using the
     * WorldModel's per-track velocity estimate) to cross the pursuit corridor —
     * dodges a moving robot before the camera ever sees it there.
     */
    private fun predictedBlocker(
        px: Double,
        py: Double,
        hdg: Double,
        targetBearingDeg: Double,
        targetDistCm: Double,
        nowNanos: Long,
    ): BallTarget? {
        for (o in world.opponents()) {
            if (o.ageSec(nowNanos) > WM_ROBOT_MAX_AGE_SEC) continue
            val fx = o.fieldXcm + o.velXcmPerSec * DODGE_LOOKAHEAD_SEC
            val fy = o.fieldYcm + o.velYcmPerSec * DODGE_LOOKAHEAD_SEC
            val dx = fx - px
            val dy = fy - py
            val dist = hypot(dx, dy)
            val bearing = DeadWheelMath.wrapDeg(Math.toDegrees(hdg) - Math.toDegrees(atan2(dy, dx)))
            if (abs(bearing - targetBearingDeg) < AVOID_ROBOT_BEARING_DEG && dist < targetDistCm) {
                return BallTarget(o.label, dist, bearing, 0.0, 0.0, 0.0)
            }
        }
        return null
    }

    /**
     * PHASE 1b: optional flower use — a live sighting first, else a fresh
     * remembered flower track from the WorldModel. Drives to
     * FLOWER_APPROACH_CM (turn-toward while approaching), stops, runs the
     * CUSTOMIZABLE flowerPullRoutine, then settles the intake. Returns true
     * when a pull was actually performed.
     */
    private fun useFlowerIfVisible(deadlineNanos: Long): Boolean {
        var positioned = false

        while (opModeIsActive() && System.nanoTime() < deadlineNanos) {
            NextMotor.motorEventLoop.poll()
            drive.updatePoseEstimate()
            vision.updateRoadRunnerPose(drive.pose)
            val now = System.nanoTime()
            val px = drive.pose.position.x * 2.54
            val py = drive.pose.position.y * 2.54
            val hdg = drive.pose.heading.toDouble()
            report() // keeps the WorldModel fed (flower tracks live here)

            val live = ballLocator.nearestFlower()
            val remembered = if (live == null) {
                world.flowers()
                    .filter { it.ageSec(now) <= DEAD_RECKON_MAX_AGE_SEC }
                    .minByOrNull { it.distanceTo(px, py) }
            } else {
                null
            }

            val distCm: Double
            val bearingDeg: Double
            val liveNow: Boolean
            if (live != null) {
                distCm = live.distanceCm
                bearingDeg = live.bearingDeg
                liveNow = true
            } else if (remembered != null) {
                val rel = remembered.relativeToRobot(px, py, hdg)
                distCm = rel.first
                bearingDeg = rel.second
                liveNow = false
            } else {
                break // nothing visible, nothing remembered: skip the phase
            }

            telemetry.addData(
                "FLOWER",
                "%.0fcm %+.1fdeg %s".format(distCm, bearingDeg, if (liveNow) "[live]" else "[memory]"),
            )
            telemetry.update()

            if (distCm <= FLOWER_APPROACH_CM + 5.0) {
                positioned = true
                break
            }

            val forward = (HiveScoringConfig.CAPTURE_RANGE_GAIN *
                (distCm - FLOWER_APPROACH_CM))
                .coerceIn(-HiveScoringConfig.ASSIST_MAX_POWER, HiveScoringConfig.ASSIST_MAX_POWER)
            val strafe = (-HiveScoringConfig.CAPTURE_BEARING_GAIN * bearingDeg)
                .coerceIn(-HiveScoringConfig.ASSIST_MAX_POWER, HiveScoringConfig.ASSIST_MAX_POWER)
            val turn = (-CAPTURE_TURN_GAIN * bearingDeg)
                .coerceIn(-CAPTURE_TURN_MAX_POWER, CAPTURE_TURN_MAX_POWER)
            drive.setDrivePowers(PoseVelocity2d(Vector2d(forward, strafe), turn))
        }

        drive.setDrivePowers(PoseVelocity2d(Vector2d(0.0, 0.0), 0.0))
        if (!positioned) return false

        // ==== CUSTOMIZABLE PULL-OUT — edit flowerPullRoutine in globals.kt ====
        flowerPullRoutine(intake, loader)

        // Give the released ball time to settle into the intake.
        intake.spinUp(1.0)
        NextMotor.motorEventLoop.poll()
        sleep(FLOWER_SETTLE_AFTER_PULL_MS)
        intake.stop()
        return true
    }

    /**
     * PHASE 3 core: servo into the hive shooting window. Live tag first; when
     * the tag flickers out of view, dead-reckon toward the WorldModel's
     * remembered hive position instead of sweeping blind. Returns true only
     * when the live tag says ALIGNED — firing is skipped otherwise.
     */
    private fun alignToHive(deadlineNanos: Long): Boolean {
        while (opModeIsActive() && System.nanoTime() < deadlineNanos) {
            NextMotor.motorEventLoop.poll()
            drive.updatePoseEstimate()
            vision.updateRoadRunnerPose(drive.pose)

            val target = hive.update()
            tagOverlay.publish(target)
            report(target)

            if (target == null) {
                val rememberedHive = if (world.hiveConfidence() >= 0.2) world.hive() else null
                if (rememberedHive != null) {
                    // Dead-reckon toward the remembered hive: translate toward
                    // it while turning to sweep the tag back into view.
                    val px = drive.pose.position.x * 2.54
                    val py = drive.pose.position.y * 2.54
                    val hdg = drive.pose.heading.toDouble()
                    val dx = rememberedHive.first - px
                    val dy = rememberedHive.second - py
                    val distCm = hypot(dx, dy)
                    val wrapped = ((Math.toDegrees(hdg) - Math.toDegrees(atan2(dy, dx)) + 180.0) % 360.0 + 360.0) % 360.0 - 180.0

                    val forward = (HiveScoringConfig.RANGE_GAIN *
                        (distCm - HiveScoringConfig.RANGE_CENTER_CM))
                        .coerceIn(-HiveScoringConfig.ASSIST_MAX_POWER, HiveScoringConfig.ASSIST_MAX_POWER)
                    val turn = (-HiveScoringConfig.BEARING_GAIN * wrapped)
                        .coerceIn(-HiveScoringConfig.ASSIST_MAX_POWER, HiveScoringConfig.ASSIST_MAX_POWER)
                    telemetry.addData("ALIGN", "tag lost — hive memory %.0fcm %+.0fdeg".format(distCm, wrapped))
                    telemetry.update()
                    drive.setDrivePowers(PoseVelocity2d(Vector2d(forward, 0.0), turn))
                } else {
                    // Nothing remembered either: sweep in place for a tag.
                    drive.setDrivePowers(
                        PoseVelocity2d(Vector2d(0.0, 0.0), -HiveScoringConfig.SEARCH_TURN_POWER),
                    )
                }
                continue
            }

            if (hive.zoneOf(target) == ScoringZone.ALIGNED) {
                drive.setDrivePowers(PoseVelocity2d(Vector2d(0.0, 0.0), 0.0))
                return true
            }

            val (forward, turn) = hive.rangeCorrection(target)
            drive.setDrivePowers(PoseVelocity2d(Vector2d(forward, 0.0), turn))
        }
        drive.setDrivePowers(PoseVelocity2d(Vector2d(0.0, 0.0), 0.0))
        return false
    }

    /**
     * PHASE 4 core: fire every held ball — one LOADER stroke per ball,
     * CONFIRMED by the flywheel RPM dip, wheels re-verified at speed between
     * balls (the intake does NOT run; the loader meters balls from the
     * hopper). Bounded by the clock's fire reserve.
     *
     * Never fires at max RPM on a vanished target: aborts safely (log +
     * telemetry) when no valid aligned target is available at firing time.
     * Returns loader-ejected [attempted] vs confirmed [confirmed] shots.
     */
    private data class FireResult(val attempted: Int, val confirmed: Int)

    private fun fireBalls(ballCount: Int): FireResult {
        BlackBox.event("phase", "name" to "4-fire")
        val fireTarget = hive.update()
        // Never blind-fire: if the target vanished after alignment (or we are
        // not in the ALIGNED zone), abort before spinning up. With no target
        // there is no distance to estimate RPM from, and shootingRpmFor(null)
        // would fall back to FLYWHEEL_MOTOR_MAX_RPM — launching balls we can't
        // account for.
        if (fireTarget == null || hive.zoneOf(fireTarget) != ScoringZone.ALIGNED) {
            BlackBox.event(
                "fire_aborted",
                "reason" to "no aligned target",
                "target" to (fireTarget?.label ?: "null"),
                "distanceCm" to fireTarget?.distanceCm,
                "bearingDeg" to fireTarget?.bearingDeg,
            )
            telemetry.addData("PHASE 4", "ABORT FIRE — no valid aligned target")
            telemetry.update()
            return FireResult(0, 0)
        }
        val rpm = ShotPredictor.adjustRpm(
            hive.shootingRpmFor(fireTarget),
            fireTarget.distanceCm,
            "auto",
        )
        flywheel.spinToRpm(rpm)
        // "hiveCm" is required by tools/shooter_refit.py — without it
        // every autonomous shot is silently unusable for curve fitting.
        BlackBox.event("fire_start", "rpm" to rpm, "hiveCm" to fireTarget?.distanceCm)

        // Wait until BOTH wheels are actually at the commanded speed
        // (the 6000 RPM motors spin up slowly) before feeding balls.
        val spinDeadline = System.nanoTime() +
            (clock.remainingSec().coerceAtMost(AUTONOMY_FIRE_RESERVE_SEC) * 1e9).toLong()
        while (opModeIsActive() && !flywheel.isAtSpeed(rpm) && System.nanoTime() < spinDeadline) {
            NextMotor.motorEventLoop.poll()
            telemetry.addData("PHASE 4", "spinning up to %.0f RPM".format(rpm))
            telemetry.addData(
                "Measured L/R",
                "%.0f / %.0f RPM".format(flywheel.measuredLeftRpm(), flywheel.measuredRightRpm()),
            )
            telemetry.update()
        }

        if (!flywheel.isAtSpeed(rpm)) {
            // Never feed balls into slow wheels: that's a weak shot
            // (and a possible stuck ball). Safe-skip instead.
            BlackBox.event("fire_aborted", "rpm" to rpm, "reason" to "wheels not at speed")
            telemetry.addData("PHASE 4", "SKIP FIRE — wheels never reached %.0f RPM".format(rpm))
            telemetry.update()
            return FireResult(0, 0)
        }

        telemetry.addData("PHASE 4", "FIRING at %.0f RPM".format(rpm))
        telemetry.update()
        val detector = ShotDetector(flywheel)
        var attempted = 0
        var confirmed = 0
        var stroking = false
        var armed = false
        val fireDeadline = System.nanoTime() +
            (clock.remainingSec().coerceAtMost(AUTONOMY_FIRE_RESERVE_SEC) * 1e9).toLong()
        while (attempted < ballCount && opModeIsActive() && System.nanoTime() < fireDeadline) {
            when {
                stroking && loader.busy -> report()
                stroking && !armed -> {
                    detector.arm()
                    armed = true
                }
                armed -> when (detector.update()) {
                    ShotDetector.State.CONFIRMED -> {
                        attempted++
                        confirmed++
                        stroking = false
                        armed = false
                        loader.retract()
                        // let the wheels recover before the next ball
                        val recoverBy = System.nanoTime() +
                            (SHOT_RECOVERY_TIMEOUT_MS * 1e6).toLong()
                        while (opModeIsActive() && !flywheel.isAtSpeed(rpm) &&
                            System.nanoTime() < recoverBy
                        ) {
                            report()
                        }
                    }
                    ShotDetector.State.MISSED -> {
                        // Ball left the loader but the RPM dip says it did not
                        // score: count the attempt (inventory drops), but do NOT
                        // count it as scored.
                        attempted++
                        stroking = false
                        armed = false
                        loader.retract()
                        val recoverBy = System.nanoTime() +
                            (SHOT_RECOVERY_TIMEOUT_MS * 1e6).toLong()
                        while (opModeIsActive() && !flywheel.isAtSpeed(rpm) &&
                            System.nanoTime() < recoverBy
                        ) {
                            report()
                        }
                    }
                    else -> report()
                }
                !loader.busy && loader.phase == LoaderPhase.HOME -> {
                    loader.stroke()
                    stroking = true
                }
                !loader.busy -> loader.retract() // settle any mid-cycle state
                else -> report()
            }
        }
        BlackBox.event("fire_done", "rpm" to rpm, "attempted" to attempted, "confirmed" to confirmed,
            "requested" to ballCount)
        return FireResult(attempted, confirmed)
    }

    /** Combined telemetry: hive range/angle, RR (fused) pose, vision health. */
    private fun report(target: HiveTarget? = null) {
        NextMotor.motorEventLoop.poll()
        val t = target ?: hive.update()
        tagOverlay.publish(t)
        hive.addTelemetry(telemetry, t)

        val p = drive.pose
        val px = p.position.x * 2.54
        val py = p.position.y * 2.54
        val hdg = p.heading.toDouble()
        val now = System.nanoTime()
        world.update(ballLocator.sightings(), px, py, hdg, now)
        world.updateHive(t, px, py, hdg, now)
        BlackBox.log(
            "record" to "auto",
            "poseXcm" to px, "poseYcm" to py, "headingRad" to hdg,
            "hiveCm" to t?.distanceCm, "hiveBearing" to t?.bearingDeg,
            "flyL" to flywheel.measuredLeftRpm(),
            "flyR" to flywheel.measuredRightRpm(),
            "batteryV" to ShotPredictor.reportHardwareMapVolts(hardwareMap),
        )
        world.logToBlackBox()

        telemetry.addData(
            "RR/FUSED pose",
            "(%.2f, %.2f) %.1f deg".format(p.position.x, p.position.y, Math.toDegrees(hdg)),
        )
        NeuralReferee.addTelemetry(telemetry)
        vision.addTelemetry(telemetry)
        telemetry.update()
    }
}
