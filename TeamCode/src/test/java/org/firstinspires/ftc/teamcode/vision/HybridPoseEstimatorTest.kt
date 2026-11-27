package org.firstinspires.ftc.teamcode.vision

import com.acmerobotics.roadrunner.Pose2d
import org.junit.Assert.assertEquals
import org.junit.Test

class HybridPoseEstimatorTest {
    private fun observation(timestampNanos: Long, confidence: Double) =
        CameraObservation(
                cameraId = CameraId.FRONT,
                timestampNanos = timestampNanos,
                fps = 30.0,
                detectedCount = 40,
                trackedCount = 40,
                inlierCount = 40,
                outlierCount = 0,
                ransacOk = true,
                inlierRatio = 1.0,
                meanTrackErrorPx = 0.0,
                confidence = confidence,
                motion = RobotMotionDelta(0.0, 0.0, 0.0, confidence, timestampNanos, true),
                features = emptyList(),
                landmarks = emptyList(),
            )

    @Test
    fun `continuous usable vision recovers from lost after the recovery window`() {
        var frontObservation: CameraObservation? = null
        val estimator = HybridPoseEstimator({ null }, { frontObservation })
        val pose = Pose2d(0.0, 0.0, 0.0)
        var now = 1_000_000_000L

        estimator.update(now, pose)
        now += 1_100_000_000L
        estimator.update(now, pose)
        assertEquals(VisionState.LOST, estimator.currentState)

        repeat(5) {
            now += 100_000_000L
            frontObservation = observation(now, 0.9)
            estimator.update(now, pose)
        }
        assertEquals(VisionState.LOST, estimator.currentState)

        now += 200_000_000L
        frontObservation = observation(now, 0.9)
        estimator.update(now, pose)
        assertEquals(VisionState.RECOVERING, estimator.currentState)
    }
}
