package org.firstinspires.ftc.teamcode.AutoAim

import dev.nextftc.core.subsystems.Subsystem
import dev.nextftc.ftc.ActiveOpMode
import org.firstinspires.ftc.teamcode.HoodConstants
import org.firstinspires.ftc.teamcode.Lower.Drive.Drive

import org.firstinspires.ftc.teamcode.Shooter.Hood.Hood
import org.firstinspires.ftc.teamcode.ShooterConstants
import org.firstinspires.ftc.teamcode.subsystem.FlyWheel

/**
 * AutoAim Class
 * Provides unified interface for automatic aiming based on distance
 *
 * This class combines:
 * - Flywheel velocity (based on distance)
 * - Hood angle (based on distance)
 * - Turret angle (from Turret subsystem)
 *
 * TUNING GUIDE:
 *
 * 1. FLYWHEEL:
 *    - Measure distance vs required RPM through testing
 *    - Create a lookup table or curve
 *    - Start with linear interpolation between presets
 *
 * 2. HOOD:
 *    - Test different distances with fixed flywheel speed
 *    - Adjust hood until shot lands in goal
 *    - Record distance → hood position pairs
 *
 * 3. INTEGRATION:
 *    - Call update() in teleOp loop
 *    - System automatically adjusts flywheel and hood
 *    - Turret aims separately using Drive data
 */
object AutoAim: Subsystem {
    // ==================== STATE ====================
    var enabled = false

    /** Current calculated distance to goal */
    var currentDistance = 0.0

    /** Current calculated RPM */
    var targetRpm = 0.0

    /** Current calculated hood position */
    var targetHoodPosition = 0.0

    // ==================== UPDATE ====================
    /**
     * Update all aim components based on current robot position
     * Call this in teleOp onUpdate() loop
     */
    fun update() {
        if (!enabled) return

        // Get distance from Drive subsystem
        currentDistance = Drive.distanceToGoalMeters()

        // Calculate flywheel RPM
        targetRpm = calculateFlywheelRpm(currentDistance)
        FlyWheel.setVelocity(targetRpm)

        // Calculate hood position
        targetHoodPosition = Hood.getPositionForDistance(currentDistance)
        Hood.setPosition(targetHoodPosition)
    }

    /**
     * Enable/disable auto aim
     */
    fun setAutoAim(enable: Boolean) {
        enabled = enable
        if (!enable) {
            // Optionally reset to defaults when disabled
        }
    }

    // ==================== FLYWHEEL CALCULATION ====================
    /**
     * Calculate required flywheel RPM from distance
     *
     * TUNING:
     * This is a placeholder implementation using linear interpolation
     * between presets. For more accuracy:
     * 1. Test at multiple distances
     * 2. Record required RPM for each
     * 3. Create a lookup table or curve fit
     *
     * @param distanceMeters Distance to goal in meters
     * @return Required RPM
     */
    private fun calculateFlywheelRpm(distanceMeters: Double): Double {
        // Linear interpolation between presets
        return when {
            distanceMeters < HoodConstants.DISTANCE_CLOSE_THRESHOLD ->
                ShooterConstants.FLYWHEEL_CLOSE_RPM

            distanceMeters < HoodConstants.DISTANCE_MID_THRESHOLD -> {
                val t = (distanceMeters - HoodConstants.DISTANCE_CLOSE_THRESHOLD) /
                        (HoodConstants.DISTANCE_MID_THRESHOLD - HoodConstants.DISTANCE_CLOSE_THRESHOLD)
                ShooterConstants.FLYWHEEL_CLOSE_RPM + t *
                        (ShooterConstants.FLYWHEEL_MID_RPM - ShooterConstants.FLYWHEEL_CLOSE_RPM)
            }

            else -> {
                // Beyond mid threshold - use far or extrapolate
                val t = (distanceMeters - HoodConstants.DISTANCE_MID_THRESHOLD) / 2.0
                ShooterConstants.FLYWHEEL_MID_RPM + t.coerceAtMost(1.0) *
                        (ShooterConstants.FLYWHEEL_FAR_RPM - ShooterConstants.FLYWHEEL_MID_RPM)
            }
        }
    }

    // ==================== SHOOTING ON THE MOVE ====================
    /**
     * Calculate aim for shooting on the move
     * Uses virtual goal position based on robot velocity
     *
     * @return Aim parameters (distance, rpm, hood)
     */
    fun calculateForSOTM(): AimParams {
        // Get virtual goal position from Drive
        val virtualX = Drive.getVirtualGoalX()
        val virtualY = Drive.getVirtualGoalY()

        // Calculate distance to virtual goal
        val dx = virtualX - Drive.currentX
        val dy = virtualY - Drive.currentY
        val distance = kotlin.math.sqrt(dx * dx + dy * dy) * 0.0254 // inches to meters

        return AimParams(
            distance = distance,
            rpm = calculateFlywheelRpm(distance),
            hoodPosition = Hood.getPositionForDistance(distance),
            angleToVirtualGoal = kotlin.math.atan2(dy, dx)
        )
    }

    // ==================== TELEMETRY ====================
    fun updateTelemetry() {
        if (enabled) {
            ActiveOpMode.telemetry.addData("AutoAim/Enabled", "YES")
            ActiveOpMode.telemetry.addData("AutoAim/Distance", "%.2fm".format(currentDistance))
            ActiveOpMode.telemetry.addData("AutoAim/RPM", "%.0f".format(targetRpm))
            ActiveOpMode.telemetry.addData("AutoAim/Hood", "%.2f".format(targetHoodPosition))
        } else {
            ActiveOpMode.telemetry.addData("AutoAim/Enabled", "NO")
        }
    }

    // ==================== DATA CLASS ====================
    data class AimParams(
        val distance: Double,        // Meters
        val rpm: Double,            // Flywheel RPM
        val hoodPosition: Double,   // Servo position 0-1
        val angleToVirtualGoal: Double  // Radians
    )
}