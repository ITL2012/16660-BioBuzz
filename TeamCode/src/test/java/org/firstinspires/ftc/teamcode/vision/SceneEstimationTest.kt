package org.firstinspires.ftc.teamcode.vision

import org.junit.Assert.assertEquals
import org.junit.Assert.assertFalse
import org.junit.Assert.assertTrue
import org.junit.Test
import kotlin.math.cos
import kotlin.math.sin

class SceneEstimationTest {
    private fun correspondenceSet(
        count: Int,
        theta: Double,
        scale: Double = 1.0,
        tx: Double = 0.0,
        ty: Double = 0.0,
    ): Pair<DoubleArray, DoubleArray> {
        val prev = DoubleArray(count * 2)
        val cur = DoubleArray(count * 2)
        val c = cos(theta) * scale
        val s = sin(theta) * scale
        for (i in 0 until count) {
            val x = 40.0 + (i % 10) * 55.0
            val y = 35.0 + (i / 10) * 60.0 + (i % 3) * 4.0
            prev[2 * i] = x
            prev[2 * i + 1] = y
            cur[2 * i] = c * x - s * y + tx
            cur[2 * i + 1] = s * x + c * y + ty
        }
        return prev to cur
    }

    @Test
    fun `two-point RANSAC preserves positive rotation`() {
        val theta = 0.12
        val (prev, cur) = correspondenceSet(40, theta, tx = 4.0, ty = -3.0)

        val result = StaticSceneEstimator().estimate(prev, cur)

        assertTrue(result.ok)
        assertEquals(theta, result.thetaRad, 1e-9)
        assertEquals(1.0, result.scale, 1e-9)
        assertEquals(4.0, result.txPx, 1e-7)
        assertEquals(-3.0, result.tyPx, 1e-7)
        assertEquals(40, result.inlierIndices.size)
    }

    @Test
    fun `negative rotation and scale are recovered`() {
        val theta = -0.08
        val (prev, cur) = correspondenceSet(40, theta, scale = 1.03, tx = -8.0, ty = 5.0)

        val result = StaticSceneEstimator().estimate(prev, cur)

        assertTrue(result.ok)
        assertEquals(theta, result.thetaRad, 1e-9)
        assertEquals(1.03, result.scale, 1e-9)
        assertEquals(-8.0, result.txPx, 1e-7)
        assertEquals(5.0, result.tyPx, 1e-7)
    }

    @Test
    fun `outliers do not corrupt the consensus transform`() {
        val theta = 0.06
        val (prev, cur) = correspondenceSet(50, theta, tx = 3.0, ty = 2.0)
        val noisyPrev = prev.copyOf(120)
        val noisyCur = cur.copyOf(120)
        for (i in 50 until 60) {
            noisyPrev[2 * i] = 50.0 + i * 7.0
            noisyPrev[2 * i + 1] = 400.0 - i * 3.0
            noisyCur[2 * i] = 500.0 - i * 11.0
            noisyCur[2 * i + 1] = 30.0 + i * 13.0
        }

        val result = StaticSceneEstimator().estimate(noisyPrev, noisyCur)

        assertTrue(result.ok)
        assertEquals(theta, result.thetaRad, 1e-6)
        assertEquals(50, result.inlierIndices.size)
        assertTrue(result.inlierIndices.all { it < 50 })
    }

    @Test
    fun `degenerate correspondences fail safely`() {
        val prev = DoubleArray(40) { 100.0 }
        val cur = DoubleArray(40) { 200.0 }

        val result = StaticSceneEstimator().estimate(prev, cur)

        assertFalse(result.ok)
        assertTrue(result.inlierIndices.isEmpty())
    }
}
