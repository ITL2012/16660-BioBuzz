package org.firstinspires.ftc.teamcode

import kotlin.random.Random
import org.junit.Assert.assertEquals
import org.junit.Assert.assertNotNull
import org.junit.Assert.assertTrue
import org.junit.Test

/**
 * JVM tests for the MCTS planner.
 *
 * The two properties that matter:
 *
 *  * the SEARCH NEVER MUTATES THE LIVE WORLD. A playout is a hypothetical
 *    future — if it were allowed to remove real balls from the WorldModel or
 *    black out real flower patches, 128 playouts per planner tick would
 *    delete the robot's actual map. The planner therefore keeps its
 *    collection bookkeeping in a private overlay.
 *  * PROGRESSIVE WIDENING keeps the branching factor bounded, so a node with
 *    twenty candidate actions does not explode the tree and starve the
 *    search of depth.
 */
class MCTSPlannerTest {

    private fun pollen(distCm: Double, bearingDeg: Double = 0.0) =
        Sighting("Pollen", distCm, bearingDeg, 500.0, isBall = true)

    private fun nectar(distCm: Double, bearingDeg: Double = 0.0) =
        Sighting("Nectar Blue", distCm, bearingDeg, 500.0, isBall = false)

    private fun world(vararg sightings: Sighting): WorldModel {
        val m = WorldModel()
        m.update(sightings.toList(), 0.0, 0.0, 0.0, 0L)
        return m
    }

    @Test
    fun `the search does not remove balls from the live world`() {
        val m = world(pollen(80.0), pollen(120.0, 20.0), pollen(200.0, -30.0))
        val before = m.balls().size
        assertEquals(3, before)

        val planner = MCTSPlanner(
            playouts = 64, maxDepth = 4, timeBudgetMs = 5000.0, random = Random(1),
        )
        planner.decide(m, 0, 25.0, 0.0, 0.0)

        assertEquals("the search deleted real balls", before, m.balls().size)
    }

    @Test
    fun `the search does not empty real flower clusters`() {
        val m = world(nectar(80.0), nectar(90.0))
        m.forceClusterRebuild(0L)
        val clustersBefore = m.flowerClusters().size
        val availableBefore = m.availableFlowerClusters(0L).size

        val planner = MCTSPlanner(
            playouts = 64, maxDepth = 4, timeBudgetMs = 5000.0, random = Random(2),
        )
        planner.decide(m, 0, 25.0, 0.0, 0.0)

        assertEquals("the search changed the real clusters", clustersBefore, m.flowerClusters().size)
        assertEquals(
            "the search depleted a real flower bed",
            availableBefore,
            m.availableFlowerClusters(0L).size,
        )
    }

    @Test
    fun `the requested number of playouts is run`() {
        val m = world(pollen(80.0))
        val planner = MCTSPlanner(
            playouts = 48, maxDepth = 3, timeBudgetMs = 10000.0, random = Random(3),
        )
        planner.decide(m, 0, 25.0, 0.0, 0.0)
        assertEquals(48, planner.lastPlayoutsDone)
        assertTrue("the search built no nodes", planner.lastNodeCount > 1)
    }

    @Test
    fun `the search respects its time budget`() {
        val m = world(pollen(80.0), pollen(120.0, 20.0), pollen(160.0, -20.0))
        val planner = MCTSPlanner(
            playouts = 100000, maxDepth = 8, timeBudgetMs = 20.0, random = Random(4),
        )
        planner.decide(m, 0, 25.0, 0.0, 0.0)
        assertTrue(
            "the search ran for ${planner.lastSearchMillis}ms with a 20ms budget",
            planner.lastSearchMillis < 2000.0,
        )
        assertTrue("the budget should have cut the playouts short", planner.lastPlayoutsDone < 100000)
    }

    @Test
    fun `progressive widening keeps the tree far smaller than the action count`() {
        // eight balls on the field: without progressive widening every root
        // node would fan out into eight children on the first playout
        val sightings = ArrayList<Sighting>()
        for (i in 0 until 8) {
            sightings.add(pollen(60.0 + i * 10.0, i * 8.0 - 30.0))
        }
        val m = world(*sightings.toTypedArray())

        val planner = MCTSPlanner(
            playouts = 32, maxDepth = 4, timeBudgetMs = 10000.0, random = Random(5),
        )
        planner.decide(m, 0, 25.0, 0.0, 0.0)

        // 32 playouts with widening: the tree stays a few dozen nodes, not
        // the 8 * 32 = 256 a full fan-out would create
        assertTrue(
            "the tree grew to ${planner.lastNodeCount} nodes",
            planner.lastNodeCount < 32 * 8,
        )
    }

    @Test
    fun `a wider coefficient builds a bigger tree`() {
        val sightings = ArrayList<Sighting>()
        for (i in 0 until 8) {
            sightings.add(pollen(60.0 + i * 10.0, i * 8.0 - 30.0))
        }
        val m = world(*sightings.toTypedArray())

        val narrow = MCTSPlanner(
            playouts = 64, maxDepth = 4,
            progressiveWideningK = 0.2, timeBudgetMs = 10000.0, random = Random(6),
        )
        narrow.decide(m, 0, 25.0, 0.0, 0.0)

        val wide = MCTSPlanner(
            playouts = 64, maxDepth = 4,
            progressiveWideningK = 4.0, timeBudgetMs = 10000.0, random = Random(6),
        )
        wide.decide(m, 0, 25.0, 0.0, 0.0)

        assertTrue(
            "wide (${wide.lastNodeCount}) should not be smaller than narrow (${narrow.lastNodeCount})",
            wide.lastNodeCount >= narrow.lastNodeCount,
        )
    }

    @Test
    fun `progressive widening follows the one-plus-k-sqrt-visits formula`() {
        val planner = MCTSPlanner(progressiveWideningK = 2.0)
        assertEquals(1, planner.maxChildrenForVisits(0))
        assertEquals(3, planner.maxChildrenForVisits(1)) // 1 + floor(2*1)
        assertEquals(7, planner.maxChildrenForVisits(9)) // 1 + floor(2*3)
        assertEquals(21, planner.maxChildrenForVisits(100)) // 1 + floor(2*10)
    }

    @Test
    fun `progressive widening never lets a node go below one child`() {
        val planner = MCTSPlanner(progressiveWideningK = 0.0)
        assertEquals(1, planner.maxChildrenForVisits(0))
        assertEquals(1, planner.maxChildrenForVisits(1000))
    }

    @Test
    fun `the widening limit grows with the visit count`() {
        val planner = MCTSPlanner(progressiveWideningK = MCTS_PROGRESSIVE_WIDENING_K)
        var previous = 0
        for (visits in listOf(0, 1, 4, 16, 64, 256)) {
            val limit = planner.maxChildrenForVisits(visits)
            assertTrue("the limit must not shrink ($visits visits)", limit >= previous)
            previous = limit
        }
    }

    @Test
    fun `the planner returns a usable decision`() {
        val m = world(pollen(80.0))
        val planner = MCTSPlanner(
            playouts = 32, maxDepth = 3, timeBudgetMs = 10000.0, random = Random(7),
        )
        val decision = planner.decide(m, 0, 25.0, 0.0, 0.0)
        assertNotNull(decision)
        assertTrue(decision.rationale.isNotEmpty())
    }

    @Test
    fun `with nothing on the field the planner does not ask to chase`() {
        val m = WorldModel()
        val planner = MCTSPlanner(
            playouts = 16, maxDepth = 3, timeBudgetMs = 10000.0, random = Random(8),
        )
        val decision = planner.decide(m, 0, 25.0, 0.0, 0.0)
        assertNotNull(decision)
        assertTrue(
            "unexpected action ${decision.action}",
            decision.action != PlannerAction.COLLECT_BALL,
        )
    }

    @Test
    fun `the default planner runs the tuned playout count`() {
        val planner = MCTSPlanner()
        assertEquals(128, planner.playouts)
        assertEquals(4, planner.maxDepth)
        assertEquals(MCTS_PROGRESSIVE_WIDENING_K, planner.progressiveWideningK, 1e-9)
        assertEquals(MCTS_HEURISTIC_WEIGHT, planner.heuristicWeight, 1e-9)
    }
}
