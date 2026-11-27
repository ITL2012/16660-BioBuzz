package org.firstinspires.ftc.teamcode

import kotlin.math.abs
import kotlin.math.cos
import kotlin.math.sin

/**
 * ============ DEAD-WHEEL MATH — pure 3-wheel odometry kinematics ============
 *
 * Shared by [org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive.DeadWheelLocalizer]
 * and the JVM unit tests. No Android imports.
 *
 * Wheel model: a wheel at robot-frame offset (x, y) with rolling direction
 * (cos θ, sin θ) reads
 *     d = cos θ·dx + sin θ·dy + (x·sin θ − y·cos θ)·δ
 * for body displacement (dx, dy, δ). Position increments solve that
 * over-determined system by least squares with δ taken from the IMU;
 * velocities solve the same equations exactly for (vx, vy, ω) via Cramer.
 *
 * All lengths in INCHES, angles in RADIANS, `measured` values must already
 * be converted to inches per tick (sign flips for reversed wheels applied).
 */
object DeadWheelMath {

    data class WheelSpec(
        /** Mounting offset from the drive center, inches (+x fwd, +y left). */
        val x: Double,
        val y: Double,
        /** Rolling direction: 0 = forward, PI/2 = strafing-left rolls it. */
        val angleRad: Double,
    )

    class Kinematics(wheels: List<WheelSpec>) {
        private val cos = DoubleArray(wheels.size)
        private val sin = DoubleArray(wheels.size)
        private val rotation = DoubleArray(wheels.size)
        private var p00 = 0.0
        private var p01 = 0.0
        private var p10 = 0.0
        private var p11 = 0.0
        private val inverse = DoubleArray(9)
        private var velocityValid = false

        init {
            require(wheels.size == 3) { "three wheels are required" }
            var sxx = 0.0
            var sxy = 0.0
            var syy = 0.0
            for (i in wheels.indices) {
                val w = wheels[i]
                cos[i] = cos(w.angleRad)
                sin[i] = sin(w.angleRad)
                rotation[i] = w.x * sin[i] - w.y * cos[i]
                sxx += cos[i] * cos[i]
                sxy += cos[i] * sin[i]
                syy += sin[i] * sin[i]
            }
            val positionDet = sxx * syy - sxy * sxy
            if (abs(positionDet) >= 1e-9) {
                p00 = syy / positionDet
                p01 = -sxy / positionDet
                p10 = -sxy / positionDet
                p11 = sxx / positionDet
            }

            val a = cos
            val b = sin
            val c = rotation
            val det = a[0] * (b[1] * c[2] - c[1] * b[2]) -
                b[0] * (a[1] * c[2] - c[1] * a[2]) +
                c[0] * (a[1] * b[2] - b[1] * a[2])
            if (abs(det) >= 1e-9) {
                velocityValid = true
                inverse[0] = (b[1] * c[2] - c[1] * b[2]) / det
                inverse[1] = (c[0] * b[2] - b[0] * c[2]) / det
                inverse[2] = (b[0] * c[1] - c[0] * b[1]) / det
                inverse[3] = (c[1] * a[2] - a[1] * c[2]) / det
                inverse[4] = (a[0] * c[2] - c[0] * a[2]) / det
                inverse[5] = (c[0] * a[1] - a[0] * c[1]) / det
                inverse[6] = (a[1] * b[2] - b[1] * a[2]) / det
                inverse[7] = (a[2] * b[0] - a[0] * b[2]) / det
                inverse[8] = (a[0] * b[1] - b[0] * a[1]) / det
            }
        }

        fun positionDelta(measuredIn: DoubleArray, headingDelta: Double): Pair<Double, Double> {
            require(measuredIn.size == cos.size) { "wheel/measurement count mismatch" }
            var bx = 0.0
            var by = 0.0
            for (i in cos.indices) {
                val measured = measuredIn[i] - rotation[i] * headingDelta
                bx += cos[i] * measured
                by += sin[i] * measured
            }
            return (p00 * bx + p01 * by) to (p10 * bx + p11 * by)
        }

        fun velocity(measuredInPerSec: DoubleArray): Triple<Double, Double, Double> {
            require(measuredInPerSec.size == 3) { "wheel/measurement count mismatch" }
            if (!velocityValid) return Triple(0.0, 0.0, 0.0)
            return Triple(
                inverse[0] * measuredInPerSec[0] + inverse[1] * measuredInPerSec[1] + inverse[2] * measuredInPerSec[2],
                inverse[3] * measuredInPerSec[0] + inverse[4] * measuredInPerSec[1] + inverse[5] * measuredInPerSec[2],
                inverse[6] * measuredInPerSec[0] + inverse[7] * measuredInPerSec[1] + inverse[8] * measuredInPerSec[2],
            )
        }
    }

    fun inchesPerTick(ticksPerRev: Double, diameterIn: Double): Double =
        Math.PI * diameterIn / ticksPerRev

    /** Reverses a measured tick delta for a backwards-mounted wheel. */
    fun signed(measured: Double, reversed: Boolean): Double = if (reversed) -measured else measured

    /**
     * Body-frame displacement (dx, dy) from the three wheel measurements
     * (already in inches, sign-corrected) and the IMU heading delta.
     * Least squares over the 3 equations, 2 unknowns.
     */
    fun positionDelta(
        wheels: List<WheelSpec>,
        measuredIn: List<Double>,
        headingDelta: Double,
    ): Pair<Double, Double> {
        require(wheels.size == measuredIn.size) { "wheel/measurement count mismatch" }
        var sxx = 0.0
        var sxy = 0.0
        var syy = 0.0
        var bx = 0.0
        var by = 0.0
        for (i in wheels.indices) {
            val w = wheels[i]
            val cosT = cos(w.angleRad)
            val sinT = sin(w.angleRad)
            // remove the arc the wheel traveled purely by rotating
            val b = measuredIn[i] - (w.x * sinT - w.y * cosT) * headingDelta
            sxx += cosT * cosT
            sxy += cosT * sinT
            syy += sinT * sinT
            bx += cosT * b
            by += sinT * b
        }
        val det = sxx * syy - sxy * sxy
        if (abs(det) < 1e-9) return 0.0 to 0.0
        return (syy * bx - sxy * by) / det to (sxx * by - sxy * bx) / det
    }

    /**
     * Body-frame velocity (vx, vy, omega) from the three wheel speeds (inches
     * per second, sign-corrected). Exact 3x3 solve by Cramer's rule; a
     * degenerate wheel layout yields zeros.
     */
    fun velocity(
        wheels: List<WheelSpec>,
        measuredInPerSec: List<Double>,
    ): Triple<Double, Double, Double> {
        require(wheels.size == measuredInPerSec.size) { "wheel/measurement count mismatch" }
        val a = DoubleArray(3) { i -> cos(wheels[i].angleRad) }
        val b = DoubleArray(3) { i -> sin(wheels[i].angleRad) }
        val c = DoubleArray(3) { i -> wheels[i].x * sin(wheels[i].angleRad) - wheels[i].y * cos(wheels[i].angleRad) }
        val r = measuredInPerSec.toDoubleArray()

        fun det3(m0: Double, m1: Double, m2: Double) =
            m0 * (b[1] * c[2] - c[1] * b[2]) -
                m1 * (b[0] * c[2] - c[0] * b[2]) +
                m2 * (b[0] * c[1] - c[0] * b[1])

        val d = det3(a[0], a[1], a[2])
        if (abs(d) < 1e-9) return Triple(0.0, 0.0, 0.0)

        // Cramer's rule: each unknown replaces its matrix column with r.
        val vx = det3(r[0], r[1], r[2]) / d
        val vy = (a[0] * (r[1] * c[2] - c[1] * r[2]) -
            r[0] * (a[1] * c[2] - c[1] * a[2]) +
            c[0] * (a[1] * r[2] - r[1] * a[2])) / d
        val omega = (a[0] * (b[1] * r[2] - r[1] * b[2]) -
            b[0] * (a[1] * r[2] - r[1] * a[2]) +
            r[0] * (a[1] * b[2] - b[1] * a[2])) / d
        return Triple(vx, vy, omega)
    }

    /** Wraps an angle in degrees into [-180, 180). */
    fun wrapDeg(a: Double): Double = ((a + 180.0) % 360.0 + 360.0) % 360.0 - 180.0
}
