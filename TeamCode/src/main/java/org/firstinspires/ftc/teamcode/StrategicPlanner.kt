package org.firstinspires.ftc.teamcode

import com.acmerobotics.dashboard.FtcDashboard
import java.util.Locale
import kotlin.math.abs
import kotlin.math.hypot
import kotlin.math.max
import kotlin.math.min
import kotlin.random.Random

/**
 * ============ STRATEGIC PLANNER — the lookahead brain ============
 *
 * Replaces the reactive "what do I see RIGHT NOW" strategy enum with a
 * value/cost search over the WorldModel's memory:
 *
 *  * every candidate action (collect ball i / go score / use flower / park)
 *    gets an expected value and an estimated time cost,
 *  * balls an opponent is closer to get discounted (they'll take them),
 *  * the ENDGAME is modeled: nothing starts that can't pay back before the
 *    clock runs out — full payloads force a score run, the last seconds
 *    force park.
 *
 * Deliberately small and deterministic (runs in microseconds) — the structure
 * is ready for a deeper search (MCTS) later, but this level already beats the
 * greedy controller because it remembers unseen balls and budgets time.
 *
 * It also powers SHADOW MODE: called during human teleop it logs what it
 * would do (without acting) — that log is how we'll prove it outplays the
 * human before ever letting it drive.
 */
class StrategicPlanner {

    /**
     * Picks the best action given the remembered world and our own state.
     *
     * @param world          persistent field memory (tracks + hive)
     * @param ballsHeld      balls currently in the robot
     * @param timeRemainingSec  seconds until the match ends — never start
     *                       anything that can't finish
     * @param robotXcm/Ycm   current robot position (field cm)
     */
    fun decide(
        world: WorldModel,
        ballsHeld: Int,
        timeRemainingSec: Double,
        robotXcm: Double,
        robotYcm: Double,
    ): PlannerDecision {
        val hive = world.hive()
        val distToHive =
            if (hive != null) hypot(hive.first - robotXcm, hive.second - robotYcm) else 150.0
        val baseScoreTime = distToHive / ROBOT_SPEED_CM_PER_SEC
        val hiveObstaclePenalty = if (hive != null && world.isPathBlocked(robotXcm, robotYcm, hive.first, hive.second)) 3.0 else 0.0
        val scoreTime = baseScoreTime + PLANNER_SCORE_TOTAL_SEC + hiveObstaclePenalty

        // ---- 1. ENDGAME: the clock is the strongest constraint ----
        // DESPERATION MODE: if time < 2*scoreTime, force score even with 1 ball,
        // and allow shooting from TOO_FAR zone (ShotPredictor will nudge RPM)
        val desperationMode = timeRemainingSec < 2.0 * scoreTime && ballsHeld > 0 && hive != null
        if (timeRemainingSec <= PLANNER_PARK_SEC + 2.0) {
            return PlannerDecision(
                PlannerAction.PARK, -1, "", 0.0, 0.0, 0.0, PLANNER_PARK_SEC,
                "PARK: %.0fs left".format(timeRemainingSec),
            )
        }
        if (ballsHeld > 0 && (timeRemainingSec <= scoreTime + PLANNER_ENDGAME_MARGIN_SEC || desperationMode)) {
            val rationale = if (desperationMode) {
                "DESPERATION score: %.0fs left, 2xScoreTime=%.1fs, balls=%d"
                    .format(timeRemainingSec, 2.0 * scoreTime, ballsHeld)
            } else {
                "ENDGAME score run: %.0fs left, need %.1fs".format(timeRemainingSec, scoreTime)
            }
            return PlannerDecision(
                PlannerAction.GO_SCORE, -1, "Hive", 0.0, 0.0,
                ballsHeld * POLLEN_VALUE, scoreTime,
                rationale,
            )
        }

        // ---- 2. FULL: no point holding more than we can carry ----
        if (ballsHeld >= MAX_BALLS_HELD && hive != null) {
            return PlannerDecision(
                PlannerAction.GO_SCORE, -1, "Hive", hive.first, hive.second,
                ballsHeld * POLLEN_VALUE, scoreTime,
                "FULL (%d balls) -> score".format(ballsHeld),
            )
        }

        // ---- 3. Evaluate collect candidates (balls, then flowers) ----
        val opponents = world.opponents()
        var bestBall: TrackedObject? = null
        var bestDensity = 0.0
        var bestTime = 0.0
        var bestEffValue = 0.0
        for (b in world.balls()) {
            val dist = b.distanceTo(robotXcm, robotYcm)
            val baseT = dist / ROBOT_SPEED_CM_PER_SEC
            val obstaclePenalty = if (world.isPathBlocked(robotXcm, robotYcm, b.fieldXcm, b.fieldYcm)) 2.0 else 0.0
            val t = baseT + PLANNER_COLLECT_HANDLE_SEC + obstaclePenalty
            var v = b.value * b.confidence
            // opponent closer to this ball than we are? discount based on their intent
            for (o in opponents) {
                if (o.distanceTo(b.fieldXcm, b.fieldYcm) < dist) {
                    val intentDiscount = when (o.intent) {
                        OpponentIntent.DEFENDING -> 0.3  // they're guarding, very likely to take
                        OpponentIntent.SCORING -> 0.5    // passing through but might grab
                        OpponentIntent.COLLECTING -> 0.4 // actively collecting
                        OpponentIntent.TRANSIT -> 0.7    // just passing through
                        OpponentIntent.UNKNOWN -> PLANNER_OPPONENT_DISCOUNT // fallback
                    }
                    v *= intentDiscount * o.intentConfidence.coerceIn(0.3, 1.0)
                    break
                }
            }
            // can't collect AND still score it afterwards?
            if (timeRemainingSec < t + scoreTime + PLANNER_ENDGAME_MARGIN_SEC) continue
            val density = v / (1.0 + t)
            if (density > bestDensity) {
                bestDensity = density; bestBall = b; bestTime = t; bestEffValue = v
            }
        }

        var bestFlower: TrackedObject? = null
        var flowerDensity = 0.0
        var flowerTime = 0.0
        var bestFlowerCluster: FlowerCluster? = null
        if (FLOWER_PULL_ENABLED && ballsHeld < MAX_BALLS_HELD) {
            val now = System.nanoTime()
            // Only clusters that are FULL and hold at least one flower that
            // is not in its depletion cooldown are worth driving to.
            for (cluster in world.availableFlowerClusters(now)) {
                val usable = cluster.flowers.count { world.flowerState(it.id, now) == FlowerState.FULL }
                if (usable <= 0) continue
                val dist = cluster.distanceTo(robotXcm, robotYcm)
                val baseT = dist / ROBOT_SPEED_CM_PER_SEC
                val obstaclePenalty = if (world.isPathBlocked(robotXcm, robotYcm, cluster.centerXcm, cluster.centerYcm)) 3.0 else 0.0
                val t = baseT + PLANNER_FLOWER_TOTAL_SEC * usable + obstaclePenalty
                // Nectar is worth 2x pollen (NECTAR_VALUE = 2.0)
                val v = usable * NECTAR_VALUE * 0.9 // confidence factor for cluster
                if (timeRemainingSec < t + scoreTime + PLANNER_ENDGAME_MARGIN_SEC) continue
                if (ballsHeld + usable > MAX_BALLS_HELD) continue // capacity check
                val density = v / (1.0 + t)
                if (density > flowerDensity) {
                    flowerDensity = density
                    bestFlowerCluster = cluster
                    flowerTime = t
                }
            }
            // Fallback to individual flowers if no clusters (backward compatibility)
            if (bestFlowerCluster == null) {
                for (fl in world.availableFlowers(now)) {
                    val dist = fl.distanceTo(robotXcm, robotYcm)
                    val baseT = dist / ROBOT_SPEED_CM_PER_SEC
                    val obstaclePenalty = if (world.isPathBlocked(robotXcm, robotYcm, fl.fieldXcm, fl.fieldYcm)) 2.0 else 0.0
                    val t = baseT + PLANNER_FLOWER_TOTAL_SEC + obstaclePenalty
                    val v = NECTAR_VALUE * fl.confidence
                    if (timeRemainingSec < t + scoreTime + PLANNER_ENDGAME_MARGIN_SEC) continue
                    val density = v / (1.0 + t)
                    if (density > flowerDensity) {
                        flowerDensity = density; bestFlower = fl; flowerTime = t
                    }
                }
            }
        }

        // Scoring what we hold is always an option; compare value densities.
        val scoreDensity =
            if (ballsHeld > 0) (ballsHeld * POLLEN_VALUE) / (1.0 + scoreTime) else 0.0

        // ---- 4. Choose ----
        // A flower candidate exists either as a cluster (preferred) or as a
        // single-flower fallback — both must be able to win.
        val flowerWins = (bestFlowerCluster != null || bestFlower != null) &&
            flowerDensity > bestDensity && flowerDensity > scoreDensity
        val ballWins = bestBall != null && bestDensity > scoreDensity
        if (ballsHeld > 0 && !ballWins && !flowerWins && hive != null) {
            return PlannerDecision(
                PlannerAction.GO_SCORE, -1, "Hive", hive.first, hive.second,
                ballsHeld * POLLEN_VALUE, scoreTime,
                "score beats collecting (%.2f vs %.2f density)".format(scoreDensity, bestDensity),
            )
        }
        if (flowerWins && bestFlowerCluster != null) {
            val targetCluster = bestFlowerCluster!!
            return PlannerDecision(
                PlannerAction.USE_FLOWER, targetCluster.id, "FlowerCluster",
                targetCluster.centerXcm, targetCluster.centerYcm,
                targetCluster.totalNectarValue, flowerTime,
                "cluster %.3f density (nectar %.1f, %d flowers)".format(flowerDensity, targetCluster.totalNectarValue, targetCluster.flowerCount),
            )
        }
        if (flowerWins && bestFlower != null) {
            val targetFlower = bestFlower!!
            return PlannerDecision(
                PlannerAction.USE_FLOWER, targetFlower.id, targetFlower.label,
                targetFlower.fieldXcm, targetFlower.fieldYcm,
                NECTAR_VALUE * targetFlower.confidence, flowerTime,
                "flower %.3f density (nectar %.1f)".format(flowerDensity, NECTAR_VALUE),
            )
        }
        if (ballWins || (bestBall != null && ballsHeld == 0)) {
            return PlannerDecision(
                PlannerAction.COLLECT_BALL, bestBall!!.id, bestBall.label,
                bestBall.fieldXcm, bestBall.fieldYcm, bestEffValue, bestTime,
                "best density %.3f (val %.1f conf %.0f%%)".format(
                    bestDensity, bestBall.value, bestBall.confidence * 100,
                ),
            )
        }
        return PlannerDecision(
            PlannerAction.HOLD, -1, "", 0.0, 0.0, 0.0, 0.0,
            "nothing worth doing — hold & scan",
        )
    }

    /** Logs a decision to the Black Box with a source tag (real / shadow). */
    fun logDecision(d: PlannerDecision, source: String, ballsHeld: Int, timeRemainingSec: Double) {
        BlackBox.decision(
            SessionDecision(
                tNanos = System.nanoTime(),
                tMs = System.currentTimeMillis(),
                action = d.action.name,
                targetId = d.targetId,
                targetLabel = d.targetLabel,
                expectedValue = d.expectedValue,
                estCostSec = d.estCostSec,
                ballsHeld = ballsHeld,
                timeRemainingSec = timeRemainingSec,
                rationale = d.rationale,
                source = source,
            ),
        )
        // Shadow Planner Dashboard: stream to FTC Dashboard for live viewing
        if (SHADOW_PLANNER_ENABLED && source == "shadow") {
            try {
                val dashboard = com.acmerobotics.dashboard.FtcDashboard.getInstance()
                dashboard?.getTelemetry()?.apply {
                    addData("Shadow Planner", d.action.name)
                    addData("Shadow Target", "${d.targetLabel} #${d.targetId}")
                    addData("Shadow Value", String.format(Locale.US, "%.1f", d.expectedValue))
                    addData("Shadow Cost", String.format(Locale.US, "%.1fs", d.estCostSec))
                    addData("Shadow Rationale", d.rationale)
                    update()
                }
            } catch (_: Exception) {
                // Dashboard not available, ignore
            }
        }
    }
}