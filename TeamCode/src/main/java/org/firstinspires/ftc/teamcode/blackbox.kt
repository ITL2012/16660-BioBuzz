package org.firstinspires.ftc.teamcode

import android.content.Context
import android.database.sqlite.SQLiteDatabase
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration
import org.firstinspires.ftc.vision.VisionProcessor
import org.opencv.core.Mat
import org.opencv.core.MatOfByte
import org.opencv.imgcodecs.Imgcodecs
import org.opencv.imgproc.Imgproc
import java.io.File
import java.util.concurrent.Executors
import java.util.concurrent.LinkedBlockingQueue
import java.util.concurrent.ThreadPoolExecutor
import java.util.concurrent.TimeUnit
import java.util.concurrent.atomic.AtomicInteger


/**
 * ============ BLACK BOX — passive full-match recorder + frame capture ============
 *
 * One merged home for the recorder pair (was the blackbox/ package):
 *  * [BlackBox]           — the session recorder
 *  * [FrameGrabProcessor] — subsampled JPEG frames from both cameras
 *
 * A session is ONE SQLite file. The robot state, the world-model tracks,
 * the planner decisions, the events AND the camera frames (as BLOBS) all
 * live in typed tables inside that one file, so copying a match off the
 * robot is copying one file — not a folder with thousands of loose JPEGs.
 *
 * Legality posture: 100% passive, 100% onboard, no new hardware — see
 * docs/LEGALITY-REVIEW.md.
 */

import android.util.Log

/**
 * ============ BLACK BOX — passive full-match data recorder ============
 *
 * Writes ONE SQLite file per OpMode run to the ROBOT CONTROLLER's own
 * external storage. The file holds the robot's state, the world model's
 * tracks, the planner decisions, the events, and the camera frames.
 *
 * LEGALITY POSTURE (see docs/LEGALITY-REVIEW.md):
 *  * 100% PASSIVE — reads state, never influences control.
 *  * 100% ONBOARD — writes only to the Robot Controller's storage; no
 *    streaming, no radio, no internet, nothing leaves the robot mid-match.
 *  * No new hardware — uses the two cameras that are already attached to the
 *    Robot Controller for the vision stack.
 * After the match, copy the .db off the phone over USB and open it with
 * SQLite (or the tools in tools/, which read it with Python's sqlite3).
 *
 * Files land in: /sdcard/Android/data/<rc-app>/files/blackbox/<label>_<ts>.db
 */
object BlackBox {

    private const val TAG = "BlackBox"

    /** True while a recording session is active. Cheap to poll from anywhere. */
    @Volatile
    var active = false
        private set

    /** The session file, once the session has started. */
    @Volatile
    var sessionFile: File? = null
        private set

    /** Approximate bytes written by frame JPEGs this session (budget guard). */
    @Volatile
    var frameBytesSaved = 0L
        private set

    /** Frames dropped because the writer could not keep up. */
    @Volatile
    var framesDropped = 0L
        private set

    /** Rows the database refused (the recorder never throws at the robot). */
    @Volatile
    var droppedRows = 0L
        private set

    private var db: SQLiteDatabase? = null
    private var store: SessionStore? = null
    private var sessionRef: Long = -1L
    private var eventSeq = 0

    /**
     * All database work happens on ONE background thread. The control loop
     * and the vision thread only hand rows over; neither ever waits on a
     * disk write.
     */
    private var writer: ThreadPoolExecutor? = null
    private val pendingFrames = AtomicInteger(0)

    /**
     * Starts a new session. Call once at the top of runOpMode().
     * [label] names the file, e.g. "teleop" / "auto" / "selfplay".
     * Returns false (and does nothing) when BLACKBOX_ENABLED is false or
     * storage is unavailable.
     */
    @Synchronized
    fun start(context: Context, label: String): Boolean {
        if (!BLACKBOX_ENABLED) return false
        stop()
        return try {
            val root = File(context.getExternalFilesDir(null) ?: context.filesDir, "blackbox")
            root.mkdirs()
            val file = File(root, "${label}_${System.currentTimeMillis()}.db")
            val database = SQLiteDatabase.openOrCreateDatabase(file, null)
            // WAL keeps the writes cheap: the frame BLOBs are appended
            // instead of forcing a disk sync per row.
            database.enableWriteAheadLogging()
            val sql = AndroidSql(database)
            val sessionStore = SessionStore(sql).createSchema()
            val ref = sessionStore.beginSession(
                sessionId = file.nameWithoutExtension,
                label = label,
                createdAtMs = System.currentTimeMillis(),
                startedNanos = System.nanoTime(),
            )
            db = database
            store = sessionStore
            sessionRef = ref
            sessionFile = file
            eventSeq = 0
            frameBytesSaved = 0
            framesDropped = 0
            droppedRows = 0
            pendingFrames.set(0)
            writer = ThreadPoolExecutor(
                1, 1, 0L, TimeUnit.MILLISECONDS, LinkedBlockingQueue(),
            ) { r -> Thread(r, "blackbox-writer").apply { isDaemon = true } }
            active = true
            event("session_start", "label" to label, "version" to SessionSchema.VERSION)
            true
        } catch (e: Exception) {
            Log.w(TAG, "black box could not start: ${e.message}")
            closeDatabase()
            active = false
            false
        }
    }

    /**
     * Appends one state snapshot. Everything is typed — there is no JSON
     * anywhere in the recorder, so a record can never be half-written.
     */
    @Synchronized
    fun state(s: SessionState) {
        val ref = sessionRef
        val st = store
        val w = writer
        if (!active || ref < 0 || st == null || w == null) return
        submit(w) {
            try {
                st.addState(ref, s)
            } catch (_: Exception) {
                droppedRows++
            }
        }
    }

    /** Appends the world-model tracks of one snapshot. */
    @Synchronized
    fun tracks(tracks: List<SessionTrack>) {
        val ref = sessionRef
        val st = store
        val w = writer
        if (!active || ref < 0 || st == null || w == null || tracks.isEmpty()) return
        val tNanos = System.nanoTime()
        submit(w) {
            try {
                st.addTracks(ref, tNanos, tracks)
            } catch (_: Exception) {
                droppedRows++
            }
        }
    }

    /**
     * Appends one planner decision.
     */
    @Synchronized
    fun decision(d: SessionDecision) {
        val ref = sessionRef
        val st = store
        val w = writer
        if (!active || ref < 0 || st == null || w == null) return
        submit(w) {
            try {
                st.addDecision(ref, d)
            } catch (_: Exception) {
                droppedRows++
            }
        }
    }

    /**
     * A state line tagged with an event type — phase changes, shots, jams...
     * The payload is stored in typed columns, so readers can filter on it
     * with plain SQL.
     */
    @Synchronized
    fun event(type: String, vararg fields: Pair<String, Any?>) {
        val ref = sessionRef
        val st = store
        val w = writer
        if (!active || ref < 0 || st == null || w == null) return
        val seq = eventSeq++
        val e = SessionEvent(System.nanoTime(), System.currentTimeMillis(), type, fields.toList())
        submit(w) {
            try {
                st.addEvent(ref, seq, e)
            } catch (_: Exception) {
                droppedRows++
            }
        }
    }

    /** False once the frame budget is exhausted (state keeps logging). */
    fun frameBudgetAvailable(): Boolean =
        active && frameBytesSaved < BLACKBOX_MAX_SESSION_MB * 1024.0 * 1024.0

    /**
     * Stores one camera frame. The JPEG bytes go straight into the database
     * as a BLOB — no loose files, no folder tree, nothing to copy but the
     * one session file.
     */
    @Synchronized
    fun frame(camera: String, captureNanos: Long, jpeg: ByteArray, widthPx: Int = 0, heightPx: Int = 0) {
        val ref = sessionRef
        val st = store
        val w = writer
        if (!active || ref < 0 || st == null || w == null) return
        // If the writer falls behind, drop frames rather than stall the
        // vision thread — a slightly gappy video beats a stalled camera.
        if (pendingFrames.get() >= MAX_PENDING_FRAMES) {
            framesDropped++
            return
        }
        pendingFrames.incrementAndGet()
        frameBytesSaved += jpeg.size
        val f = SessionFrame(camera, captureNanos, jpeg, widthPx, heightPx, System.currentTimeMillis())
        submit(w) {
            try {
                st.addFrame(ref, f)
            } catch (_: Exception) {
                droppedRows++
            } finally {
                pendingFrames.decrementAndGet()
            }
        }
    }

    /** Ends the session, flushes everything to disk and closes the file. */
    @Synchronized
    fun stop() {
        if (!active && db == null) {
            sessionFile = null
            return
        }
        val w = writer
        writer = null
        if (w != null) {
            w.shutdown()
            try {
                w.awaitTermination(5, TimeUnit.SECONDS)
            } catch (_: InterruptedException) {
            }
        }
        val st = store
        val ref = sessionRef
        if (st != null && ref >= 0) {
            try {
                st.endSession(ref, System.nanoTime())
            } catch (_: Exception) {
            }
        }
        closeDatabase()
        store = null
        sessionRef = -1L
        sessionFile = null
        active = false
    }

    private fun closeDatabase() {
        val d = db
        db = null
        try {
            if (d != null) {
                // Fold the write-ahead log back into the session file, so a
                // finished match is ONE .db and not a .db plus a .db-wal.
                d.rawQuery("PRAGMA wal_checkpoint(TRUNCATE)", null).use { c ->
                    c.moveToFirst()
                }
                d.close()
            }
        } catch (_: Exception) {
        }
    }

    /**
     * Hands a row to the writer thread. A rejected task means the queue is
     * full — the row is dropped rather than blocking the caller.
     */
    private fun submit(w: ThreadPoolExecutor, task: () -> Unit) {
        try {
            w.execute(task)
        } catch (_: Exception) {
            droppedRows++
        }
    }

    /** How many frames may be waiting for the writer at once. */
    private const val MAX_PENDING_FRAMES = 8
}


/**
 * Attaches to a VisionPortal alongside the normal vision processors and saves
 * subsampled JPEG frames from the raw camera stream into the Black Box
 * session. The frames are the EXACT images the vision stack saw — their
 * capture timestamps (captureNanos) match the state rows, so a computer can
 * replay what the robot "was thinking" frame by frame.
 *
 * Cost control: BLACKBOX_FRAME_CAPTURE_FPS per camera and the session MB
 * budget in globals.kt. Runs on the vision thread and swallows its own errors
 * so it can never stall image processing.
 */
class FrameGrabProcessor(private val camera: String) : VisionProcessor {

    @Volatile
    var enabled = true
    private var lastSaveNanos = 0L
    private var frameCount = 0

    override fun init(width: Int, height: Int, calibration: CameraCalibration?) {}

    override fun processFrame(frame: Mat?, captureTimeNanos: Long): Any? {
        val f = frame ?: return null
        if (!enabled || !FRAME_CAPTURE_ENABLED || !BlackBox.active) return null
        val intervalNanos = (1e9 / BLACKBOX_FRAME_CAPTURE_FPS).toLong()
        if (captureTimeNanos - lastSaveNanos < intervalNanos) return null
        if (!BlackBox.frameBudgetAvailable()) return null

        lastSaveNanos = captureTimeNanos
        var bgr: Mat? = null
        var encoded: Mat? = null
        return try {
            val bgrMat = Mat()
            // VisionProcessor frames are 4-channel RGBA; encoding them as
            // BGRA would silently save R/B-SWAPPED JPEGs (poisoning the
            // offline HSV auto-labeling). Convert to BGR first.
            Imgproc.cvtColor(f, bgrMat, Imgproc.COLOR_RGBA2BGR)
            bgr = bgrMat
            val buf = MatOfByte()
            encoded = buf
            // Encode straight into memory — the bytes go into the database,
            // there is no file to write and no folder to copy.
            if (Imgcodecs.imencode(".jpg", bgrMat, buf)) {
                val jpeg = buf.toArray()
                if (jpeg.isNotEmpty()) {
                    BlackBox.frame(
                        camera = camera,
                        captureNanos = captureTimeNanos,
                        jpeg = jpeg,
                        widthPx = bgrMat.cols(),
                        heightPx = bgrMat.rows(),
                    )
                    frameCount++
                }
            }
            null
        } catch (_: Exception) {
            null
        } finally {
            bgr?.release()
            encoded?.release()
        }
    }

    override fun onDrawFrame(
        canvas: android.graphics.Canvas?,
        onscreenWidth: Int,
        onscreenHeight: Int,
        scaleBmpPxToCanvasPx: Float,
        scaleCanvasDensity: Float,
        userContext: Any?,
    ) {
    }
}

/**
 * SUPERHUMAN-ONLY: dumps the core [WorldModel]'s snapshot to the Black Box
 * (~10 Hz). The core class stays Black-Box-free (identical to master); the
 * branch adds the logging as an extension so every call site is unchanged.
 */
fun WorldModel.logToBlackBox() {
    val hivePos = hive()
    BlackBox.state(
        SessionState(
            tNanos = System.nanoTime(),
            tMs = System.currentTimeMillis(),
            hiveXcm = hivePos?.first,
            hiveYcm = hivePos?.second,
            hiveConfidence = hiveConfidence(),
        ),
    )
    BlackBox.tracks(
        balls().map {
            SessionTrack(
                kind = "ball", trackId = it.id, xCm = it.fieldXcm, yCm = it.fieldYcm,
                label = it.label, confidence = it.confidence,
            )
        } + opponents().map {
            SessionTrack(
                kind = "opponent", trackId = it.id, xCm = it.fieldXcm, yCm = it.fieldYcm,
                label = it.label, confidence = it.confidence,
                velXcmPerSec = it.velXcmPerSec, velYcmPerSec = it.velYcmPerSec,
                intent = it.intent.name, intentConfidence = it.intentConfidence,
            )
        } + flowers().map {
            SessionTrack(
                kind = "flower", trackId = it.id, xCm = it.fieldXcm, yCm = it.fieldYcm,
                label = it.label, confidence = it.confidence,
            )
        },
    )
}
