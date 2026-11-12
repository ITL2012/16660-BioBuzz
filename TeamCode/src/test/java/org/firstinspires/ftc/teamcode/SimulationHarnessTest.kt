package org.firstinspires.ftc.teamcode

import org.junit.Assert.assertEquals
import org.junit.Assert.assertTrue
import org.junit.Assert.fail
import org.junit.Test
import java.io.File
import java.nio.file.Files
import java.sql.DriverManager

/**
 * Tests for the simulation harness itself — the replay and the golden
 * master.
 *
 * The harness is the regression net for the whole autonomy stack: replay a
 * recorded session, let the planner react, and compare the result against a
 * golden master. If the planner's behaviour changes, the golden check fails
 * with a readable diff instead of the team discovering it on the field.
 *
 * The session and the golden master are SQLite files now — the same format
 * the Robot Controller records — so there is no JSON to misparse anywhere in
 * this path.
 */
class SimulationHarnessTest {

    private fun tmpFile(name: String): File =
        Files.createTempDirectory("biobuzz-harness").resolve(name).toFile()

    @Test
    fun `the replay produces planner decisions`() {
        val session = tmpFile("session.db")
        SessionFixture.writeSession(session)
        val golden = tmpFile("goldens.db")

        val harness = SimulationHarness().loadSession(session).run()
        harness.exportGoldenMaster(golden, name = "harness")

        assertTrue(golden.exists())
        assertTrue("the golden master should not be empty", golden.length() > 0)
        assertTrue("the replay should have decided something", harness.decisionCount() > 0)
    }

    @Test
    fun `a replay matches its own golden master`() {
        val session = tmpFile("session.db")
        SessionFixture.writeSession(session)
        val golden = tmpFile("goldens.db")

        SimulationHarness().loadSession(session).run().exportGoldenMaster(golden, name = "harness")

        // Replaying the same session must reproduce the same decisions.
        SimulationHarness().loadSession(session).run().assertPlannerMatchesGolden(golden, name = "harness")
    }

    @Test
    fun `the golden check reports a mismatch instead of failing silently`() {
        val session = tmpFile("session.db")
        SessionFixture.writeSession(session)
        val golden = tmpFile("goldens.db")

        SimulationHarness().loadSession(session).run().exportGoldenMaster(golden, name = "harness")

        // Corrupt the FIRST decision of the golden master: whatever the
        // planner chose, the check must notice.
        val conn = DriverManager.getConnection("jdbc:sqlite:${golden.absolutePath}")
        try {
            conn.createStatement().use { st ->
                st.executeUpdate(
                    "UPDATE golden_decision SET action = 'BOGUS' " +
                        "WHERE id = (SELECT MIN(id) FROM golden_decision)",
                )
            }
        } finally {
            conn.close()
        }

        try {
            SimulationHarness().loadSession(session).run().assertPlannerMatchesGolden(golden, name = "harness")
            fail("a corrupted golden master must not pass the check")
        } catch (e: AssertionError) {
            assertTrue(
                "the failure should explain the drift, got: ${e.message}",
                e.message?.contains("drifted from the golden master") == true,
            )
        }
    }

    @Test
    fun `a missing golden master is reported clearly`() {
        val session = tmpFile("session.db")
        SessionFixture.writeSession(session)
        try {
            SimulationHarness().loadSession(session).run()
                .assertPlannerMatchesGolden(File("/nonexistent/goldens.db"), name = "harness")
            fail("a missing golden master must not pass the check")
        } catch (e: IllegalArgumentException) {
            assertTrue(e.message?.contains("golden master not found") == true)
        }
    }

    @Test
    fun `an unknown golden name is reported clearly`() {
        val session = tmpFile("session.db")
        SessionFixture.writeSession(session)
        val golden = tmpFile("goldens.db")
        SimulationHarness().loadSession(session).run().exportGoldenMaster(golden, name = "harness")

        try {
            SimulationHarness().loadSession(session).run()
                .assertPlannerMatchesGolden(golden, name = "does-not-exist")
            fail("an unknown golden must not pass the check")
        } catch (e: IllegalArgumentException) {
            assertTrue(e.message?.contains("golden master not found") == true)
        }
    }

    @Test
    fun `a row with a bad value is reported instead of vanishing`() {
        val session = tmpFile("session.db")
        val (store, conn) = SessionFixture.open(session)
        val ref = try {
            SessionFixture.writeSession(session)
            store.sessions().first().let { (it["id"] as Number).toLong() }
        } finally {
            conn.close()
        }
        // A row whose numeric column holds text: the replay must skip it
        // and say so, instead of blowing up the whole session.
        val conn2 = DriverManager.getConnection("jdbc:sqlite:${session.absolutePath}")
        try {
            conn2.prepareStatement(
                "INSERT INTO track (session_ref, t_nanos, kind, track_id, x_cm, y_cm, label, confidence) " +
                    "VALUES (?, ?, 'ball', 99, 'not-a-number', 0.0, 'Ball', 0.5)",
            ).use { ps ->
                ps.setLong(1, ref)
                ps.setLong(2, 2_000_000_000L)
                ps.executeUpdate()
            }
        } finally {
            conn2.close()
        }

        val harness = SimulationHarness().loadSession(session).run()
        assertTrue(
            "the bad row should be reported, events: ${harness.events()}",
            harness.events().any { it.contains("bad_row") },
        )
        assertTrue("the replay should still have produced decisions", harness.decisionCount() > 0)
    }

    @Test
    fun `the replay covers the whole recorded session`() {
        val session = tmpFile("session.db")
        SessionFixture.writeSession(session)

        val harness = SimulationHarness().loadSession(session).run()
        // The recording spans 2.5 s; the planner runs at 2 Hz, so there must
        // be several decisions.
        assertTrue("expected several decisions, got ${harness.decisionCount()}", harness.decisionCount() >= 3)
    }

    @Test
    fun `the session's frames survive in the same file`() {
        val session = tmpFile("session.db")
        val (store, conn) = SessionFixture.open(session)
        try {
            val ref = store.beginSession("frames-test")
            val jpeg = byteArrayOf(0xFF.toByte(), 0xD8.toByte(), 1, 2, 3, 0xFF.toByte(), 0xD9.toByte())
            store.addFrame(ref, SessionFrame("FRONT", 1_500_000_000L, jpeg, 640, 480))
            store.endSession(ref, 2_000_000_000L)
        } finally {
            conn.close()
        }

        val (reader, conn2) = SessionFixture.open(session)
        try {
            val ref = reader.sessions().first().let { (it["id"] as Number).toLong() }
            val frames = reader.frames(ref)
            assertEquals("the frame should be in the database", 1, frames.size)
            val data = frames[0]["data"] as ByteArray
            assertEquals("the JPEG bytes should survive the round trip", 7, data.size)
            assertEquals("FRONT", frames[0]["camera"])
        } finally {
            conn2.close()
        }
    }

    @Test
    fun `a session and its goldens can live in the same file`() {
        val session = tmpFile("session.db")
        SessionFixture.writeSession(session)

        // The session file itself can hold the golden master too.
        SimulationHarness().loadSession(session).run()
            .exportGoldenMaster(session, name = "inline")

        SimulationHarness().loadSession(session).run()
            .assertPlannerMatchesGolden(session, name = "inline")
    }
}
