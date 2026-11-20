package org.firstinspires.ftc.teamcode

import kotlin.math.abs
import kotlin.math.max

/**
 * ============ DRIVE CONFIG — one table for the whole drivetrain ============
 *
 * EVERYTHING about how the four drive motors are wired lives here:
 *   * which Robot Config name each wheel has   (names live in globals.kt)
 *   * the direction of each wheel               (the four *_REVERSED flags)
 *   * the mixing matrix that turns driver intent into wheel commands
 *
 * Why one file (lesson from the pushbot-simple branch): the same physical
 * drivetrain is driven by TWO code paths — the NextFTC [Drivetrain] class
 * (teleop / bench OpModes) and the Road Runner [roadrunner.MecanumDrive]
 * (autonomous / localization). When those two paths each carried their own
 * copy of the directions and the mixing matrix, they drifted apart and the
 * robot drove one way in teleop and a different way in autonomous. Both paths
 * now read THIS table, so they cannot disagree.
 *
 * DIRECTION SEMANTICS (verified on the real robot with the "Wheel Direction
 * Test" OpMode — see README Chapter 3):
 *   Each flag is that motor's `DcMotorSimple.Direction`. On a mecanum the
 *   two sides are MIRRORED, so the right-side motors are mounted backwards
 *   relative to the left. The flags below compensate for that, which is
 *   what makes the standard mixing matrix work:
 *
 *     commanding +1 to ALL FOUR wheels  ->  the robot drives FORWARD
 *     FL and BL spin one way, FR and BR spin the OTHER way
 *
 *   That is the whole point: "FL spins the same way as FR" is WRONG on a
 *   mecanum. The defaults below are the layout this robot was built with
 *   (and the layout the pushbot-simple branch proved on the same
 *   drivetrain): left wheels FORWARD, right wheels REVERSE.
 *
 *   If the robot strafes when you push forward, or spins when you push
 *   forward, ONE of these four flags is wrong. Flip exactly one flag here,
 *   redeploy, and re-run "Wheel Direction Test" — do NOT rewire the hub.
 *
 * MIXING (identical to the proven JJ TestTeleOP scheme, and to the Road
 * Runner mecanum kinematics with the same axis signs):
 *
 *   forward +Y, strafe +X = RIGHT, turn + = RIGHT (clockwise)
 *
 *     FL = forward + strafe + turn
 *     FR = forward - strafe - turn
 *     BL = forward - strafe + turn
 *     BR = forward + strafe - turn
 *
 *   scaled by 1/(|forward| + |strafe| + |turn|) so no wheel is ever asked for
 *   more than full power.
 */
object DriveConfig {

    // ---- per-wheel polarity (the ONLY place drive polarity is decided) ----

    /**
     * Front-left motor direction. `false` = FORWARD, `true` = REVERSE.
     * See the class docs: on this mirrored mecanum the two sides use opposite
     * polarities, and commanding +1 to all four drives the robot forward.
     */
    const val FL_REVERSED = false

    /** Front-right motor direction. `false` = FORWARD, `true` = REVERSE. */
    const val FR_REVERSED = true

    /** Back-left motor direction. `false` = FORWARD, `true` = REVERSE. */
    const val BL_REVERSED = false

    /** Back-right motor direction. `false` = FORWARD, `true` = REVERSE. */
    const val BR_REVERSED = true

    /**
     * Mixes driver intent into the four wheel commands.
     *
     * @param forward +forward / -backward, in [-1, 1]
     * @param strafe  +right / -left, in [-1, 1]
     * @param turn    +right (clockwise) / -left (counter-clockwise), in [-1, 1]
     * @return wheel commands in the order FL, FR, BL, BR, each in [-1, 1]
     */
    fun mix(forward: Double, strafe: Double, turn: Double): DoubleArray {
        val denom = max(abs(forward) + abs(strafe) + abs(turn), 1.0)
        return doubleArrayOf(
            (forward + strafe + turn) / denom, // FL
            (forward - strafe - turn) / denom, // FR
            (forward - strafe + turn) / denom, // BL
            (forward + strafe - turn) / denom, // BR
        )
    }

    /** Human-readable summary for the Driver Station / manual cross-checks. */
    fun summary(): List<Pair<String, String>> = listOf(
        "frontleft" to "${DrivetrainFrontLeft} (${if (FL_REVERSED) "REVERSE" else "FORWARD"})",
        "frontright" to "${DrivetrainFrontRight} (${if (FR_REVERSED) "REVERSE" else "FORWARD"})",
        "backleft" to "${DrivetrainBackLeft} (${if (BL_REVERSED) "REVERSE" else "FORWARD"})",
        "backright" to "${DrivetrainBackRight} (${if (BR_REVERSED) "REVERSE" else "FORWARD"})",
    )
}
