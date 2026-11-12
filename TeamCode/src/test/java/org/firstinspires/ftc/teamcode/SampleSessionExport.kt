package org.firstinspires.ftc.teamcode

import org.junit.Assert.assertTrue
import org.junit.Test
import java.io.File
import java.util.Base64

/**
 * Writes a small, realistic session database to `build/sample-session.db`.
 *
 * It exists for two reasons: the Python tools in `tools/` need something to
 * read (CI runs them against this file), and it keeps the "the robot wrote
 * it, Python reads it" path honest — the frames here are real JPEGs, so
 * `cv2.imdecode` has something it can really decode.
 */
class SampleSessionExport {

    /**
     * A real 64x48 JPEG, embedded as base64 so the file works on any
     * machine without an image library. It is a valid JPEG, so the Python
     * side can really decode it.
     */
    private fun jpeg(): ByteArray = Base64.getDecoder().decode(SAMPLE_JPEG_BASE64)

    @Test
    fun `export a sample session for the python tools`() {
        val out = File("build/sample-session.db")
        out.parentFile?.mkdirs()
        if (out.exists()) out.delete()

        val (store, conn) = SessionFixture.open(out)
        try {
            val ref = store.beginSession("sample-auto", "auto")
            val base = System.currentTimeMillis()
            var nanos = 1_000_000_000L

            repeat(8) { i ->
                nanos += 200_000_000L
                val tMs = base + i * 200L
                store.addState(
                    ref,
                    SessionState(
                        tNanos = nanos,
                        tMs = tMs,
                        poseXcm = 10.0 * i,
                        poseYcm = 2.0 * i,
                        headingRad = 0.05 * i,
                        hiveXcm = 150.0,
                        hiveYcm = 0.0,
                        hiveConfidence = 0.85,
                        ballsHeld = i % 3,
                        flywheelLeftRpm = 400.0 + i * 50,
                        flywheelRightRpm = 400.0 + i * 50,
                        batteryVolts = 12.8 - i * 0.05,
                    ),
                )
                store.addTracks(
                    ref,
                    nanos,
                    listOf(
                        SessionTrack("ball", 1, 50.0 + 3.0 * i, 10.0, "Pollen", 0.9),
                        SessionTrack("flower", 2, 90.0, -30.0, "Flower", 0.95),
                        SessionTrack("opponent", 3, 120.0, 40.0, "Robot", 0.8, 12.0, -3.0, "SCORING", 0.7),
                    ),
                )
                store.addFrame(
                    ref,
                    SessionFrame("FRONT", nanos, jpeg(), 64, 48, tMs),
                )
                store.addFrame(
                    ref,
                    SessionFrame("UP", nanos, jpeg(), 64, 48, tMs),
                )
            }

            store.addEvent(
                ref, 0,
                SessionEvent(nanos, base, "phase", listOf("name" to "1-collect")),
            )
            store.addEvent(
                ref, 1,
                SessionEvent(nanos, base + 10, "pickup", listOf("label" to "Pollen", "how" to "intake")),
            )
            store.addDecision(
                ref,
                SessionDecision(
                    tNanos = nanos,
                    tMs = base + 20,
                    simTimeSec = 12.5,
                    action = "GOTO",
                    targetId = 1,
                    targetLabel = "Pollen",
                    expectedValue = 12.5,
                    estCostSec = 3.0,
                    ballsHeld = 1,
                    timeRemainingSec = 28.5,
                    robotXcm = 70.0,
                    robotYcm = 14.0,
                    rationale = "closest ball",
                    source = "strategic",
                ),
            )
            store.endSession(ref, nanos)
        } finally {
            conn.close()
        }

        assertTrue("the sample session was not written", out.exists() && out.length() > 0)
        println("sample session: ${out.absolutePath}")
    }

    private companion object {
        /** A 64x48 gradient JPEG (931 bytes) — see the class comment. */
        const val SAMPLE_JPEG_BASE64 =
            "/9j/4AAQSkZJRgABAQAAAQABAAD/2wBDAAYEBQYFBAYGBQYHBwYIChAKCgkJChQODwwQFxQYGBcUFhYaHSUfGhsjHBYWICwgIyYnKSopGR8tMC0oMCUoKSj/2wBDAQcHBwoIChMKChMoGhYaKCgoKCgoKCgoKCgoKCgoKCgoKCgoKCgoKCgoKCgoKCgoKCgoKCgoKCgoKCgoKCgoKCj/wAARCAAwAEADASIAAhEBAxEB/8QAHwAAAQUBAQEBAQEAAAAAAAAAAAECAwQFBgcICQoL/8QAtRAAAgEDAwIEAwUFBAQAAAF9AQIDAAQRBRIhMUEGE1FhByJxFDKBkaEII0KxwRVS0fAkM2JyggkKFhcYGRolJicoKSo0NTY3ODk6Q0RFRkdISUpTVFVWV1hZWmNkZWZnaGlqc3R1dnd4eXqDhIWGh4iJipKTlJWWl5iZmqKjpKWmp6ipqrKztLW2t7i5usLDxMXGx8jJytLT1NXW19jZ2uHi4+Tl5ufo6erx8vP09fb3+Pn6/8QAHwEAAwEBAQEBAQEBAQAAAAAAAAECAwQFBgcICQoL/8QAtREAAgECBAQDBAcFBAQAAQJ3AAECAxEEBSExBhJBUQdhcRMiMoEIFEKRobHBCSMzUvAVYnLRChYkNOEl8RcYGRomJygpKjU2Nzg5OkNERUZHSElKU1RVVldYWVpjZGVmZ2hpanN0dXZ3eHl6goOEhYaHiImKkpOUlZaXmJmaoqOkpaanqKmqsrO0tba3uLm6wsPExcbHyMnK0tPU1dbX2Nna4uPk5ebn6Onq8vP09fb3+Pn6/9oADAMBAAIRAxEAPwD51itParsVp7VpxWntVyK09q9+eJObC4vzM2G06cVditPatKK09quxWnTiuOpiT6TC4vzMyG06cVditPatOK09quRWnTiuOeJPpcLi/MzYbTpxV2K09q0orT2q7Fae1cU8SfS4XF+ZwsVp7VditPatKK09quxWntW9TEn814XFmZFae1XYrT2rTitParkNp04rjniT6TC4szYbTpxV2K09q0orT2q7DadOK46mJPpcLizMhtOnFXYrT2rTitParsNp04riqYk+kwuLOEitParsVp7Vpw2nTirsVp7VvPEn814XF+ZmRWntV2K09q0orT2q7Fae1cdTEn0uFxfmZsVp04q5Fae1acVp7VditOnFcU8SfSYXF+ZmQ2nTirsVp7VpRWntV2K06cVxzxJ9LhcX5n//2Q=="
    }
}
