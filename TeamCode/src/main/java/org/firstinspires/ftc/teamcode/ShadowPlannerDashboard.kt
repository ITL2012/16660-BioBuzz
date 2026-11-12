package org.firstinspires.ftc.teamcode

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.acmerobotics.roadrunner.Pose2d
import com.acmerobotics.roadrunner.Vector2d
import java.util.Locale
import kotlin.math.abs
import kotlin.math.hypot
import kotlin.math.max
import org.firstinspires.ftc.teamcode.MCTSPlanner

/**
 * ============ SHADOW PLANNER DASHBOARD — live FTC Dashboard stream ============
 *
 * Streams the MCTS/Strategic planner's internal state to FTC Dashboard in real-time:
 *  - Current decision + rationale
 * - Value density curves for all candidate actions (bar chart)
 * - Opponent intent heatmap (position + intent + confidence)
 * - Time budget breakdown (park reserve, fire reserve, collect time)
 * - World model summary (balls, flowers, opponents, hive)
 *
 * Usage:
 *   val dashboard = ShadowPlannerDashboard()
 *   dashboard.update(world, ballsHeld, timeRemaining, robotX, robotY, plannerDecision)
 *
 * Enable with SHADOW_PLANNER_ENABLED in globals.kt.
 */
class ShadowPlannerDashboard {

    private var lastUpdateNanos = 0L
    private val UPDATE_INTERVAL_MS = 500 // 2 Hz dashboard updates

    /**
     * Updates the dashboard with current planner state.
     * Call from the main loop (autonomous or teleop shadow mode).
     */
    fun update(
        world: WorldModel,
        ballsHeld: Int,
        timeRemainingSec: Double,
        robotXcm: Double,
        robotYcm: Double,
        decision: PlannerDecision?,
        planner: MCTSPlanner? = null,
    ) {
        val now = System.nanoTime()
        if (now - lastUpdateNanos < UPDATE_INTERVAL_MS * 1_000_000L) return
        lastUpdateNanos = now

        if (!SHADOW_PLANNER_ENABLED) return

        try {
            val telemetry = FtcDashboard.getInstance().getTelemetry()

            // --- 1. Current Decision ---
            decision?.let { d ->
                telemetry.addData("Shadow/Action", d.action.name)
                telemetry.addData("Shadow/Target", "${d.targetLabel} #${d.targetId}")
                telemetry.addData("Shadow/Value", String.format(Locale.US, "%.1f", d.expectedValue))
                telemetry.addData("Shadow/Cost", String.format(Locale.US, "%.1fs", d.estCostSec))
                telemetry.addData("Shadow/Rationale", d.rationale)
            }

            // --- 2. Value Density Breakdown (all candidates) ---
            if (planner != null) {
                // We can't easily access internal candidates from MCTSPlanner,
                // but we can approximate by re-running generateActions
                val candidates = generateActionCandidates(world, ballsHeld, timeRemainingSec, robotXcm, robotYcm)
                val densities = candidates.map { c ->
                    val density = c.expectedValue / (1.0 + c.estCostSec)
                    c.targetLabel to density
                }.toMap()

                telemetry.addData("Shadow/ValueDensities", densities.entries
                    .sortedByDescending { it.value }
                    .joinToString(", ") { "${it.key}=${String.format(Locale.US, "%.2f", it.value)}" })
            }

            // --- 3. Opponent Intent Heatmap ---
            val oppSummary = world.opponents().map { o ->
                val dist = o.distanceTo(robotXcm, robotYcm)
                val intent = o.intent.name
                val conf = String.format(Locale.US, "%.0f%%", o.intentConfidence * 100)
                "${o.label}@${dist.toInt()}cm: $intent ($conf)"
            }.joinToString(" | ")
            telemetry.addData("Shadow/Opponents", oppSummary)

            // --- 4. Time Budget Breakdown ---
            val clock = PhaseClock()
            // We can't easily access the actual clock, but we can compute reserves
            val hive = world.hive()
            val distToHive = if (hive != null) hypot(hive.first - robotXcm, hive.second - robotYcm) else 150.0
            val scoreTime = distToHive / ROBOT_SPEED_CM_PER_SEC + PLANNER_SCORE_TOTAL_SEC
            val parkReserve = 2.5
            val fireReserve = if (hive != null) {
                val estRpm = ShooterModel.rpmForDistance(distToHive)
                val spinupSec = 1.5 + (estRpm / 6000.0) * 1.5
                val feedSec = 0.8 * 4.0
                spinupSec + feedSec + 1.5
            } else 8.0
            val totalReserve = parkReserve + fireReserve
            val collectTime = timeRemainingSec - totalReserve

            telemetry.addData("Shadow/TimeBudget", String.format(Locale.US,
                "Total: %.1fs | Collect: %.1fs | Fire: %.1fs | Park: %.1fs",
                timeRemainingSec, max(0.0, collectTime), fireReserve, parkReserve))

            // --- 5. World Model Summary ---
            telemetry.addData("Shadow/World", world.summary())

            // --- 6. Planner Config ---
            planner?.let { p: MCTSPlanner ->
                telemetry.addData("Shadow/MCTS", "playouts=${p.playouts} depth=${p.maxDepth}")
            }

            telemetry.update()
        } catch (_: Exception) {
            // Dashboard not available
        }
    }

    /**
     * Generates action candidates for dashboard visualization.
     * Mirrors MCTSPlanner.generateActions but returns the full list.
     */
    private fun generateActionCandidates(
        world: WorldModel,
        ballsHeld: Int,
        timeRemaining: Double,
        robotX: Double,
        robotY: Double,
    ): List<PlannerDecision> {
        val candidates = mutableListOf<PlannerDecision>()
        val hive = world.hive()
        val distToHive = if (hive != null) hypot(hive.first - robotX, hive.second - robotY) else 150.0
        val scoreTime = distToHive / ROBOT_SPEED_CM_PER_SEC + PLANNER_SCORE_TOTAL_SEC
        val opponents = world.opponents()

        // PARK
        if (timeRemaining <= PLANNER_PARK_SEC + 2.0) {
            candidates.add(PlannerDecision(PlannerAction.PARK, -1, "", 0.0, 0.0, 0.0, PLANNER_PARK_SEC, "PARK"))
        }

        // GO_SCORE
        if (ballsHeld > 0 && hive != null) {
            candidates.add(PlannerDecision(PlannerAction.GO_SCORE, -1, "Hive", hive.first, hive.second,
                ballsHeld * POLLEN_VALUE, scoreTime, "SCORE"))
        }

        // COLLECT_BALL
        for (b in world.balls()) {
            if (b.ageSec(System.nanoTime()) > DEAD_RECKON_MAX_AGE_SEC) continue
            if (b.lastAreaPx < CHASE_MIN_CONTOUR_AREA_PX) continue
            val dist = b.distanceTo(robotX, robotY)
            val t = dist / ROBOT_SPEED_CM_PER_SEC + PLANNER_COLLECT_HANDLE_SEC
            if (timeRemaining < t + scoreTime + PLANNER_ENDGAME_MARGIN_SEC) continue
            var v = b.value * b.confidence
            for (o in opponents) {
                if (o.distanceTo(b.fieldXcm, b.fieldYcm) < dist) {
                    val intentDiscount = when (o.intent) {
                        OpponentIntent.DEFENDING -> 0.3
                        OpponentIntent.SCORING -> 0.5
                        OpponentIntent.COLLECTING -> 0.4
                        OpponentIntent.TRANSIT -> 0.7
                        OpponentIntent.UNKNOWN -> PLANNER_OPPONENT_DISCOUNT
                    }
                    v *= intentDiscount * o.intentConfidence.coerceIn(0.3, 1.0)
                    break
                }
            }
            candidates.add(PlannerDecision(PlannerAction.COLLECT_BALL, b.id, b.label, b.fieldXcm, b.fieldYcm, v, t, "COLLECT"))
        }

        // USE_FLOWER
        if (FLOWER_PULL_ENABLED && ballsHeld < MAX_BALLS_HELD) {
            for (fl in world.flowers()) {
                val dist = fl.distanceTo(robotX, robotY)
                val t = dist / ROBOT_SPEED_CM_PER_SEC + PLANNER_FLOWER_TOTAL_SEC
                if (timeRemaining < t + scoreTime + PLANNER_ENDGAME_MARGIN_SEC) continue
                val v = POLLEN_VALUE * fl.confidence
                candidates.add(PlannerDecision(PlannerAction.USE_FLOWER, fl.id, fl.label, fl.fieldXcm, fl.fieldYcm, v, t, "FLOWER"))
            }
        }

        // HOLD
        candidates.add(PlannerDecision(PlannerAction.HOLD, -1, "", 0.0, 0.0, 0.0, 0.5, "HOLD"))

        return candidates
    }
}