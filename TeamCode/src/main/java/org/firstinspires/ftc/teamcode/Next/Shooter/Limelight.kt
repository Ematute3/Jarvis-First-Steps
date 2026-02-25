package org.firstinspires.ftc.teamcode.Shooter.Limelight

import dev.nextftc.core.subsystems.Subsystem
import dev.nextftc.ftc.ActiveOpMode

/**
 * Limelight Helper Class
 * Provides access to limelight data for turret alignment
 *
 * USAGE:
 * - Call update() each loop to refresh values
 * - Use isValidTarget() to check if seeing AprilTag
 * - Use tx, ty, ta for targeting data
 *
 * NOTE: This is a placeholder - actual limelight integration
 * depends on your specific limelight configuration and
 * AprilTag layout
 */
object Limelight : Subsystem{
    // ==================== STATE ====================
    /** Horizontal offset from crosshair (-27 to +27 degrees) */
    var tx = 0.0

    /** Vertical offset from crosshair (-20.5 to +20.5 degrees) */
    var ty = 0.0

    /** Target area (0-100%) */
    var ta = 0.0

    /** Is currently tracking valid target */
    var isTracking = false

    /** Current target ID */
    var targetId = -1

    // ==================== UPDATE ====================
    /**
     * Update limelight values from network tables
     * Call this in periodic loop
     */
    fun update() {
        // NOTE: This uses the standard Limelight network table keys
        // You may need to adjust table names based on your configuration

        try {
            // Get values from NetworkTables
            // tx - horizontal offset
            // ty - vertical offset
            // ta - target area
            // tv - valid target (0 or 1)

            // Example using standard Limelight NT keys:
            // val networkTable = NetworkTableInstance.getDefault().getTable("limelight")
            // tx = networkTable.getEntry("tx").getDouble(0.0)
            // ty = networkTable.getEntry("ty").getDouble(0.0)
            // ta = networkTable.getEntry("ta").getDouble(0.0)
            // val tv = networkTable.getEntry("tv").getDouble(0.0)
            // isTracking = tv > 0.0

            // Placeholder - replace with actual NetworkTables code
            // For now, return default values
            tx = 0.0
            ty = 0.0
            ta = 0.0
            isTracking = false
            targetId = -1

        } catch (e: Exception) {
            // If limelight not connected, default to no tracking
            isTracking = false
        }
    }

    // ==================== CHECKS ====================
    /** Check if limelight has valid target */
    fun isValidTarget(): Boolean = isTracking && targetId >= 0

    /** Check if target is within tolerance of center */
    fun isCentered(tolerance: Double = 2.0): Boolean {
        return isTracking && kotlin.math.abs(tx) < tolerance
    }

    // ==================== TELEMETRY ====================
    fun updateTelemetry() {
        ActiveOpMode.telemetry.addData("Limelight/Tracking", if (isTracking) "YES" else "NO")
        ActiveOpMode.telemetry.addData("Limelight/TX", "%.1f".format(tx))
        ActiveOpMode.telemetry.addData("Limelight/TY", "%.1f".format(ty))
        ActiveOpMode.telemetry.addData("Limelight/TA", "%.1f".format(ta))
        ActiveOpMode.telemetry.addData("Limelight/Target ID", targetId)
    }
}