package org.firstinspires.ftc.teamcode.AutoAim

import com.bylazar.configurables.annotations.Configurable
import dev.nextftc.core.subsystems.Subsystem
import dev.nextftc.ftc.ActiveOpMode
import org.firstinspires.ftc.teamcode.HoodConstants
import org.firstinspires.ftc.teamcode.Lower.Drive.Drive
import org.firstinspires.ftc.teamcode.Shooter.Hood.Hood
import org.firstinspires.ftc.teamcode.ShooterConstants
import org.firstinspires.ftc.teamcode.Next.Shooter.FlyWheel
import kotlin.math.*

@Configurable
object AutoAim : Subsystem {
    // ==================== STATE ====================
    var enabled = false

    /** Use SOTM when robot is moving above this threshold (inches/sec) */
    @JvmField var sotmVelocityThreshold = 6.0

    /** Current calculated distance to goal (inches) */
    var currentDistanceInches = 0.0

    /** Current calculated flywheel velocity (ticks/s) */
    var targetVelocity = 0.0

    /** Current calculated hood position */
    var targetHoodPosition = 0.0

    // ==================== INITIALIZATION ====================
    override fun initialize() {
        enabled = false
        currentDistanceInches = 0.0
        targetVelocity = 0.0
        targetHoodPosition = 0.0
    }

    // ==================== PERIODIC ====================
    override fun periodic() {
        if (!enabled) return

        // Check if robot is moving fast enough for SOTM
        val robotSpeed = sqrt(Drive.velocityX * Drive.velocityX + Drive.velocityY * Drive.velocityY)
        val useSOTM = robotSpeed > sotmVelocityThreshold

        if (useSOTM) {
            // Shooting on the move — aim at virtual goal
            currentDistanceInches = Drive.distanceToVirtualGoal()
        } else {
            // Stationary — aim at actual goal
            currentDistanceInches = Drive.distanceToGoal()
        }

        val distanceMeters = currentDistanceInches * 0.0254

        // Calculate and apply flywheel velocity
        targetVelocity = calculateFlywheelVelocity(distanceMeters)
        FlyWheel.setVelocity(targetVelocity)

        // Calculate and apply hood position
        targetHoodPosition = Hood.getPositionForDistance(distanceMeters)
        Hood.setPosition(targetHoodPosition)

        // Telemetry
        ActiveOpMode.telemetry.addData("AutoAim/Enabled", "YES")
        ActiveOpMode.telemetry.addData("AutoAim/SOTM", if (useSOTM) "YES" else "NO")
        ActiveOpMode.telemetry.addData("AutoAim/Distance", "%.1f in".format(currentDistanceInches))
        ActiveOpMode.telemetry.addData("AutoAim/Velocity", "%.0f t/s".format(targetVelocity))
        ActiveOpMode.telemetry.addData("AutoAim/Hood", "%.2f".format(targetHoodPosition))
    }

    // ==================== CONTROL ====================
    fun setAutoAim(enable: Boolean) {
        enabled = enable
    }

    // ==================== FLYWHEEL CALCULATION ====================
    /**
     * Calculate required flywheel velocity (ticks/s) from distance
     *
     * Uses linear interpolation between close/mid/far presets.
     * Distance thresholds are in meters to match HoodConstants.
     *
     * @param distanceMeters Distance to goal in meters
     * @return Required velocity in ticks/s
     */
    private fun calculateFlywheelVelocity(distanceMeters: Double): Double {
        return when {
            distanceMeters < HoodConstants.DISTANCE_CLOSE_THRESHOLD -> {
                // Under close range — use close preset
                ShooterConstants.FLYWHEEL_CLOSE_RPM
            }

            distanceMeters < HoodConstants.DISTANCE_MID_THRESHOLD -> {
                // Between close and mid — interpolate
                val t = (distanceMeters - HoodConstants.DISTANCE_CLOSE_THRESHOLD) /
                        (HoodConstants.DISTANCE_MID_THRESHOLD - HoodConstants.DISTANCE_CLOSE_THRESHOLD)
                ShooterConstants.FLYWHEEL_CLOSE_RPM + t *
                        (ShooterConstants.FLYWHEEL_MID_RPM - ShooterConstants.FLYWHEEL_CLOSE_RPM)
            }

            else -> {
                // Beyond mid — interpolate toward far, clamped
                val range = HoodConstants.DISTANCE_MID_THRESHOLD * 0.5
                val t = ((distanceMeters - HoodConstants.DISTANCE_MID_THRESHOLD) / range)
                    .coerceIn(0.0, 1.0)
                ShooterConstants.FLYWHEEL_MID_RPM + t *
                        (ShooterConstants.FLYWHEEL_FAR_RPM - ShooterConstants.FLYWHEEL_MID_RPM)
            }
        }
    }

    // ==================== DATA CLASS ====================
    data class AimParams(
        val distanceInches: Double,
        val velocity: Double,          // ticks/s
        val hoodPosition: Double,      // servo position 0-1
        val angleToGoal: Double        // radians
    )
}