package org.firstinspires.ftc.teamcode

import dev.nextftc.hardware.actuators.NextMotor
import kotlin.system.measureNanoTime

/**
 * ============ CONTROL FRAME — batched motor polling at fixed rate ============
 *
 * Ensures NextMotor.motorEventLoop.poll() is called exactly once per ~20 ms
 * control frame instead of redundantly in every sub-loop. All OpMode loops
 * (autonomous phases, teleop, test modes) should call [ControlFrame.tick()]
 * at the top of their main loop.
 *
 * Usage:
 *   while (opModeIsActive()) {
 *       ControlFrame.tick()
 *       // ... rest of loop
 *   }
 *
 * The frame also exposes [dtSec] for time-based calculations that need the
 * actual elapsed time since the last poll (not wall time).
 */
object ControlFrame {

    private const val TARGET_FRAME_MS = 20.0
    private const val TARGET_FRAME_NANOS = (TARGET_FRAME_MS * 1_000_000).toLong()

    @Volatile
    private var lastPollNanos = 0L

    @Volatile
    var dtSec = TARGET_FRAME_MS / 1000.0

    /** Call once per loop iteration. Polls motor event loop at most once per frame. */
    fun tick(): Double {
        val now = System.nanoTime()
        if (lastPollNanos == 0L) {
            lastPollNanos = now
            dtSec = TARGET_FRAME_MS / 1000.0
        } else {
            dtSec = (now - lastPollNanos) / 1e9
        }

        // Only poll if enough time has passed (rate limiting)
        if (now - lastPollNanos >= TARGET_FRAME_NANOS) {
            NextMotor.motorEventLoop.poll()
            lastPollNanos = now
        }

        return dtSec
    }

    /** Force an immediate poll (e.g., after changing motor setpoints that need instant effect). */
    fun forcePoll() {
        NextMotor.motorEventLoop.poll()
        lastPollNanos = System.nanoTime()
        dtSec = TARGET_FRAME_MS / 1000.0
    }

    /** Reset the frame timer (call at OpMode start). */
    fun reset() {
        lastPollNanos = 0L
        dtSec = TARGET_FRAME_MS / 1000.0
    }

    /** Blocking wait until the next frame boundary (useful for precise timing). */
    fun waitForNextFrame() {
        val target = lastPollNanos + TARGET_FRAME_NANOS
        val now = System.nanoTime()
        if (now < target) {
            Thread.sleep((target - now) / 1_000_000)
        }
        tick()
    }
}