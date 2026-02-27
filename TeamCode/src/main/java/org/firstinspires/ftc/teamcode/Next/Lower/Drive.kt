package org.firstinspires.ftc.teamcode.Lower.Drive

import com.pedropathing.geometry.Pose

import dev.nextftc.core.subsystems.Subsystem
import dev.nextftc.extensions.pedro.PedroComponent
import dev.nextftc.extensions.pedro.PedroComponent.Companion.follower
import dev.nextftc.ftc.ActiveOpMode
import org.firstinspires.ftc.teamcode.FieldConstants
import org.firstinspires.ftc.teamcode.SOTMConstants
import kotlin.math.*

/**
 * Drive Subsystem
 * Provides all position and aiming data from Pedro pathing
 *
 * This is the central source of truth for:
 * - Robot position (x, y, heading)
 * - Distance to goals
 * - Angle to goals
 * - Shooting zone detection
 * - Shooting on the move calculations
 */
object Drive : Subsystem {
    // ==================== STATE ====================
    /** Current robot position from Pedro */
    var currentX = 0.0
    var currentY = 0.0
    var currentHeading = 0.0

    /** Previous position for velocity calculation */
    private var lastX = 0.0
    private var lastY = 0.0
    private var lastTime = 0L

    /** Robot velocity */
    var velocityX = 0.0
    var velocityY = 0.0

    /** Is pose valid (Pedro has good localization) */
    var poseValid = false

    /** Last known good pose (for initialization) */
    var lastKnown = Pose(72.0, 72.0, 0.0)

    // ==================== ALLIANCE ====================
    enum class Alliance { RED, BLUE }
    var alliance = Alliance.BLUE

    // ==================== GOAL DATA ====================
    /** Get goal X based on alliance */
    val goalX: Double get() = if (alliance == Alliance.RED) FieldConstants.RED_GOAL_X else FieldConstants.BLUE_GOAL_X

    /** Goal Y (same for both alliances) */
    val goalY = FieldConstants.GOAL_Y

    // ==================== INITIALIZATION ====================
    override fun initialize() {
        // Initialize with last known pose
        follower.setStartingPose(lastKnown)
    }

    // ==================== UPDATE ====================
    /**
     * Call this in teleOp update loop to refresh all position data
     */
    fun update() {

            val pose = follower.pose
            currentX = pose.x
            currentY = pose.y
            currentHeading = pose.heading
            poseValid = true
            lastKnown = pose

            // Calculate velocity
            val now = System.currentTimeMillis()
            val dt = (now - lastTime) / 1000.0
            if (dt > 0 && dt < 0.2) {
                velocityX = (currentX - lastX) / dt
                velocityY = (currentY - lastY) / dt
            }
            lastX = currentX
            lastY = currentY
            lastTime = now

    }

    // ==================== DISTANCE CALCULATIONS ====================
    /** Distance to goal in inches */
    fun distanceToGoal(): Double {
        val dx = goalX - currentX
        val dy = goalY - currentY
        return sqrt(dx * dx + dy * dy)
    }

    /** Distance to goal in meters */
    fun distanceToGoalMeters(): Double = distanceToGoal() * 0.0254

    /** Distance to both goals */
    fun distanceToRedGoal(): Double {
        val dx = FieldConstants.RED_GOAL_X - currentX
        val dy = FieldConstants.GOAL_Y - currentY
        return sqrt(dx * dx + dy * dy)
    }

    fun distanceToBlueGoal(): Double {
        val dx = FieldConstants.BLUE_GOAL_X - currentX
        val dy = FieldConstants.GOAL_Y - currentY
        return sqrt(dx * dx + dy * dy)
    }

    // ==================== ANGLE CALCULATIONS ====================
    /** Angle to goal in field frame (degrees) */
    fun angleToGoal(): Double {
        val dx = goalX - currentX
        val dy = goalY - currentY
        return Math.toDegrees(atan2(dy, dx))
    }

    /** Angle to goal in robot frame (considering turret offset) */
    fun angleToGoalRobotFrame(): Double {
        val fieldAngle = angleToGoal()
        val robotHeadingDeg = Math.toDegrees(currentHeading)
        var turretAngle = fieldAngle - robotHeadingDeg - 90.0 // -90 for turret offset
        return normalizeAngleDegrees(turretAngle)
    }

    // ==================== SHOOTING ZONE CHECK ====================
    /** Check if robot is in main shooting zone */
    fun isInShootingZone(): Boolean {
        return FieldConstants.isInMainShootingZone(currentX, currentY)
    }

    /** Check if robot is in any shooting zone */
    fun isInAnyShootingZone(): Boolean {
        return FieldConstants.isInMainShootingZone(currentX, currentY) ||
                FieldConstants.isInSecondaryShootingZone(currentX, currentY)
    }

    // ==================== SHOOTING ON THE MOVE ====================
    /**
     * Calculate virtual goal position for shooting on the move
     * Math: virtualGoal = actualGoal + robotVelocity * timeOfFlight
     */
    fun getVirtualGoalX(): Double {
        return goalX + velocityX * SOTMConstants.TIME_OF_FLIGHT
    }

    fun getVirtualGoalY(): Double {
        return goalY + velocityY * SOTMConstants.TIME_OF_FLIGHT
    }

    /** Get angle to virtual goal */
    fun angleToVirtualGoal(): Double {
        val vx = getVirtualGoalX()
        val vy = getVirtualGoalY()
        val dx = vx - currentX
        val dy = vy - currentY
        return Math.toDegrees(atan2(dy, dx))
    }

    /** Get distance to virtual goal */
    fun distanceToVirtualGoal(): Double {
        val vx = getVirtualGoalX()
        val vy = getVirtualGoalY()
        val dx = vx - currentX
        val dy = vy - currentY
        return sqrt(dx * dx + dy * dy)
    }

    // ==================== UTILITIES ====================
    private fun normalizeAngleDegrees(degrees: Double): Double {
        var angle = degrees % 360
        if (angle > 180) angle -= 360
        if (angle < -180) angle += 360
        return angle
    }

    // ==================== PERIODIC ====================
    override fun periodic() {
        update()

        // Telemetry
        ActiveOpMode.telemetry.run {
            addData("=== DRIVE ===", "")
            addData("Drive/Valid", if (poseValid) "YES" else "NO")
            if (poseValid) {
                addData("Drive/X", "%.1f".format(currentX))
                addData("Drive/Y", "%.1f".format(currentY))
                addData("Drive/Heading", "%.1f°".format(Math.toDegrees(currentHeading)))
                addData("Drive/In Zone", if (isInShootingZone()) "YES" else "NO")
                addData("Drive/Dist Goal", "%.1f\"".format(distanceToGoal()))
                addData("Drive/Angle Goal", "%.1f°".format(angleToGoal()))
            }
        }
    }
}