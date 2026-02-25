package org.firstinspires.ftc.teamcode.Shooter.Hood

import dev.nextftc.core.commands.utility.InstantCommand
import dev.nextftc.core.subsystems.Subsystem
import dev.nextftc.hardware.impl.ServoEx
import dev.nextftc.ftc.ActiveOpMode
import org.firstinspires.ftc.teamcode.HoodConstants
import org.firstinspires.ftc.teamcode.Lower.Drive.Drive
import kotlin.math.*

/**
 * Hood Subsystem
 * Controls shooting angle based on distance to goal
 *
 * PRESETS:
 * - CLOSE: For shots < 1m (near goal)
 * - MID: For shots 1-2m
 * - FAR: For shots > 2m
 *
 * AUTO ADJUST:
 * - Automatically selects preset based on distance
 * - Can be overridden manually
 */
object Hood : Subsystem {
    // ==================== HARDWARE ====================
    private var hoodServo =ServoEx("hood")

    // ==================== STATE ====================
    enum class HoodMode {
        MANUAL,    // Manual position control
        AUTO       // Distance-based automatic
    }

    enum class HoodPreset {
        CLOSE,
        MID,
        FAR
    }

    var currentMode = HoodMode.AUTO
    var currentPreset = HoodPreset.CLOSE
    var currentPosition = HoodConstants.HOOD_CLOSE
    var targetPosition = HoodConstants.HOOD_CLOSE

    // ==================== INITIALIZATION ====================
    override fun initialize() {

        close() // Start at close position
    }

    // ==================== PRESETS ====================
    /** Set hood to close position */
    val close = InstantCommand{
        setPosition(HoodConstants.HOOD_CLOSE)
        currentPreset = HoodPreset.CLOSE
    }

    /** Set hood to mid position */
    val mid = InstantCommand{
        setPosition(HoodConstants.HOOD_MID)
        currentPreset = HoodPreset.MID
    }


    /** Set hood to far position */
    val far = InstantCommand{
        setPosition(HoodConstants.HOOD_FAR)
        currentPreset = HoodPreset.FAR
    }


    // ==================== MANUAL CONTROL ====================
    /**
     * Set exact servo position (0-1)
     * @param position Servo position 0.0 to 1.0
     */
    fun setPosition(position: Double) {
        currentMode = HoodMode.MANUAL
        targetPosition = position.coerceIn(HoodConstants.servoMinPosition, HoodConstants.servoMaxPosition)
        currentPosition = targetPosition
        hoodServo.position = targetPosition
    }

    /**
     * Set position by degrees (converts to servo position)
     * @param angleDegrees Angle in degrees
     */
    fun setAngle(angleDegrees: Double) {
        // Convert degrees to servo position
        // Assuming linear relationship between angle and servo
        // TODO: VERIFY - This is placeholder, actual relationship may be non-linear
        val t = (angleDegrees - 20.0) / 30.0 // Assuming 20-50 degree range
        val position = t.coerceIn(0.0, 1.0)
        setPosition(position)
    }

    // ==================== AUTO MODE ====================
    /**
     * Set hood automatically based on distance
     * Uses presets: close, mid, far
     */
    fun autoAdjust() {
        currentMode = HoodMode.AUTO
        adjustFromDistance()
    }

    /**
     * Adjust hood based on current distance to goal
     */
    private fun adjustFromDistance() {
        if (!Drive.poseValid) return

        val distance = Drive.distanceToGoalMeters()

        val position = when {
            distance < HoodConstants.DISTANCE_CLOSE_THRESHOLD -> {
                currentPreset = HoodPreset.CLOSE
                HoodConstants.HOOD_CLOSE
            }
            distance < HoodConstants.DISTANCE_MID_THRESHOLD -> {
                currentPreset = HoodPreset.MID
                // Interpolate between close and mid
                val t = (distance - HoodConstants.DISTANCE_CLOSE_THRESHOLD) /
                        (HoodConstants.DISTANCE_MID_THRESHOLD - HoodConstants.DISTANCE_CLOSE_THRESHOLD)
                HoodConstants.HOOD_CLOSE + t * (HoodConstants.HOOD_MID - HoodConstants.HOOD_CLOSE)
            }
            else -> {
                currentPreset = HoodPreset.FAR
                // Interpolate between mid and far
                val t = (distance - HoodConstants.DISTANCE_MID_THRESHOLD) / 2.0
                HoodConstants.HOOD_MID + t.coerceAtMost(1.0) * (HoodConstants.HOOD_FAR - HoodConstants.HOOD_MID)
            }
        }

        targetPosition = position
        currentPosition = position
        hoodServo.position = position
    }

    // ==================== AUTO AIM HELPER ====================
    /**
     * Get hood position for auto aim (called by AutoAim class)
     * @param distanceMeters Distance to goal in meters
     * @return Servo position (0-1)
     */
    fun getPositionForDistance(distanceMeters: Double): Double {
        return when {
            distanceMeters < HoodConstants.DISTANCE_CLOSE_THRESHOLD -> HoodConstants.HOOD_CLOSE
            distanceMeters < HoodConstants.DISTANCE_MID_THRESHOLD -> {
                val t = (distanceMeters - HoodConstants.DISTANCE_CLOSE_THRESHOLD) /
                        (HoodConstants.DISTANCE_MID_THRESHOLD - HoodConstants.DISTANCE_CLOSE_THRESHOLD)
                HoodConstants.HOOD_CLOSE + t * (HoodConstants.HOOD_MID - HoodConstants.HOOD_CLOSE)
            }
            else -> {
                val t = (distanceMeters - HoodConstants.DISTANCE_MID_THRESHOLD) / 2.0
                HoodConstants.HOOD_MID + t.coerceAtMost(1.0) * (HoodConstants.HOOD_FAR - HoodConstants.HOOD_MID)
            }
        }
    }

    /**
     * Set hood for specific distance (for AutoAim)
     * @param distanceMeters Distance to goal in meters
     */
    fun setForDistance(distanceMeters: Double) {
        val position = getPositionForDistance(distanceMeters)
        setPosition(position)
    }

    // ==================== PHYSICS CALCULATION ====================
    /**
     * Calculate hood angle using projectile motion physics
     * Formula: θ = arctan((v² - g*d) / (g * sqrt(d² + h²)))
     *
     * @param distanceMeters Horizontal distance to goal
     * @param velocityMps Launch velocity in m/s
     * @return Required launch angle in degrees
     */
    fun calculateAnglePhysics(distanceMeters: Double, velocityMps: Double): Double {
        val g = 9.81 // m/s²
        val h = HoodConstants.GOAL_HEIGHT - HoodConstants.SHOOTER_HEIGHT
        val d = distanceMeters
        val v = velocityMps

        val v2 = v * v
        val term1 = v2 - g * d
        val term2 = g * sqrt(d * d + h * h)

        if (term2 == 0.0) return 20.0 // Default to minimum angle

        val angleRad = atan(term1 / term2)
        return Math.toDegrees(angleRad).coerceIn(20.0, 50.0)
    }

    // ==================== PERIODIC ====================
    override fun periodic() {
        // Auto-adjust if in auto mode
        if (currentMode == HoodMode.AUTO) {
            adjustFromDistance()
        }

        // Telemetry
        ActiveOpMode.telemetry.addData("Hood/Position", "%.2f".format(currentPosition))
        ActiveOpMode.telemetry.addData("Hood/Preset", currentPreset.name)
        ActiveOpMode.telemetry.addData("Hood/Mode", currentMode.name)
    }
}