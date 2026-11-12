package org.firstinspires.ftc.teamcode

import java.io.File
import java.sql.DriverManager

/**
 * Builds Black Box session databases for the tests.
 *
 * The fixture writes the very same tables the Robot Controller writes, so
 * the harness is exercised against the real file format — not against a
 * stand-in.
 */
object SessionFixture {

    /** One instant of the recording: where the robot was and what it saw. */
    data class Snapshot(
        val xCm: Double,
        val yCm: Double,
        val headingRad: Double,
        val tracks: List<SessionTrack>,
    )


    /** Opens a session database and returns its store. */
    fun open(file: File): Pair<SessionStore, java.sql.Connection> {
        file.parentFile?.mkdirs()
        val conn = DriverManager.getConnection("jdbc:sqlite:${file.absolutePath}")
        val store = SessionStore(JdbcSql(conn)).createSchema()
        return store to conn
    }

    /**
     * A short synthetic session: the robot starts, sees a few balls and
     * flowers, and picks one up.
     */
    fun writeSession(file: File, sessionId: String = "harness-self-test"): Long {
        val (store, conn) = open(file)
        try {
            val ref = store.beginSession(sessionId, "harness-self-test")
            var tn = 1_000_000_000L
            val baseMs = System.currentTimeMillis()
            fun next(): Long {
                tn += 500_000_000L // 0.5 s between frames
                return tn
            }

            /** The wall-clock stamp that goes with a monotonic stamp. */
            fun wallMs(nanos: Long): Long = baseMs + (nanos - 1_000_000_000L) / 1_000_000L

            store.addEvent(
                ref, 0,
                SessionEvent(next(), wallMs(next()), "session_start", listOf("label" to "harness-self-test")),
            )

            // Three snapshots: two balls and a flower, then one ball gone.
            val snapshots = listOf(
                Snapshot(0.0, 0.0, 0.0, listOf(
                    SessionTrack("ball", 1, 40.0, 10.0, "Ball", 0.9),
                    SessionTrack("ball", 2, -35.0, 20.0, "Ball", 0.8),
                    SessionTrack("flower", 3, 70.0, -40.0, "Flower", 0.9),
                )),
                Snapshot(5.0, 1.0, 0.05, listOf(
                    SessionTrack("ball", 1, 45.0, 11.0, "Ball", 0.9),
                    SessionTrack("ball", 2, -30.0, 20.0, "Ball", 0.85),
                    SessionTrack("flower", 3, 75.0, -38.0, "Flower", 0.9),
                )),
                Snapshot(12.0, 2.0, 0.1, listOf(
                    SessionTrack("ball", 2, -25.0, 19.0, "Ball", 0.85),
                    SessionTrack("flower", 3, 80.0, -36.0, "Flower", 0.9),
                )),
            )

            var seq = 1
            for (snap in snapshots) {
                val t = next()
                store.addState(
                    ref,
                    SessionState(
                        tNanos = t,
                        tMs = wallMs(t),
                        poseXcm = snap.xCm,
                        poseYcm = snap.yCm,
                        headingRad = snap.headingRad,
                        hiveXcm = 90.0,
                        hiveYcm = 60.0,
                        hiveConfidence = 0.9,
                    ),
                )
                store.addTracks(ref, t, snap.tracks)
            }

            store.addEvent(
                ref, seq++,
                SessionEvent(next(), wallMs(tn), "pickup", listOf("label" to "Ball", "how" to "intake")),
            )
            store.addEvent(ref, seq, SessionEvent(next(), wallMs(tn), "cycle_start", emptyList()))
            store.endSession(ref, tn)
            return ref
        } finally {
            conn.close()
        }
    }
}
