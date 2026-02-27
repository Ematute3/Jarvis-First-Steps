package org.firstinspires.ftc.teamcode

import com.bylazar.configurables.annotations.Configurable
import com.pedropathing.geometry.Pose
import dev.nextftc.control.feedback.PIDCoefficients
import dev.nextftc.control.feedforward.BasicFeedforwardParameters
import kotlin.math.*

/**
 * Global Constants for Revival Robot
 *
 * MEASUREMENT GUIDE:
 * - Use calipers for precise measurements
 * - Use scale for weight measurements
 * - Use protractor for angle measurements
 */

// ==================== FIELD CONSTANTS ====================

@Configurable
object FieldConstants {
    /** Field size in inches */
    const val FIELD_SIZE = 144.0

    /** Goal positions */
    const val GOAL_Y = 144.0
    const val RED_GOAL_X = 144.0
    const val BLUE_GOAL_X = 0.0

    /**
     * Shooting zones (triangle coordinates)
     * Main zone: (0,144) - (144,144) - (72,72)
     * Secondary zone: (96,0) - (48,0) - (72,24)
     */


    /** Check if point is in main shooting triangle */
    fun isInMainShootingZone(x: Double, y: Double): Boolean {
        // Triangle: (0,144), (144,144), (72,72)
        val x1 = 0.0; val y1 = 144.0
        val x2 = 144.0; val y2 = 144.0
        val x3 = 72.0; val y3 = 72.0
        return barycentric(x, y, x1, y1, x2, y2, x3, y3) >= 0
    }

    /** Check if point is in secondary shooting triangle */
    fun isInSecondaryShootingZone(x: Double, y: Double): Boolean {
        // Triangle: (96,0), (48,0), (72,24)
        val x1 = 96.0; val y1 = 0.0
        val x2 = 48.0; val y2 = 0.0
        val x3 = 72.0; val y3 = 24.0
        return barycentric(x, y, x1, y1, x2, y2, x3, y3) >= 0
    }

    /** Barycentric coordinate check for point in triangle */
    private fun barycentric(px: Double, py: Double,
                            x1: Double, y1: Double,
                            x2: Double, y2: Double,
                            x3: Double, y3: Double): Double {
        val denom = (y2 - y3) * (x1 - x3) + (x3 - x2) * (y1 - y3)
        if (denom == 0.0) return -1.0
        val a = ((y2 - y3) * (px - x3) + (x3 - x2) * (py - y3)) / denom
        val b = ((y3 - y1) * (px - x3) + (x1 - x3) * (py - y3)) / denom
        val c = 1 - a - b
        return minOf(a, b, c)
    }
}

// ==================== SHOOTER CONSTANTS ====================
@Configurable
object ShooterConstants {
    /** Flywheel presets (RPM) */
    const val FLYWHEEL_CLOSE_RPM = 1000.0
    const val FLYWHEEL_MID_RPM = 1500.0
    const val FLYWHEEL_FAR_RPM = 1900.0

    /**
     * Flywheel physical properties - MEASURE THESE
     * Use calipers to measure wheel diameter
     * Use scale to measure mass
     */

    @JvmField
    var flywheelDiameter = 0.0762 // meters

    @JvmField
    var flywheelMass = 0.500 // kg

    val flywheelRadius: Double get() = flywheelDiameter / 2.0
    val momentOfInertia: Double get() = 0.5 * flywheelMass * flywheelRadius * flywheelRadius


    /**
     * PID/FF Coefficients - TUNE THESE
     *
     * TUNING GUIDE:
     * 1. Start with KV only (set KP, KI, KD to 0)
     * 2. Increase KV until flywheel reaches target but oscillates slightly
     * 3. Add KP to reduce overshoot (start at 0.01, increase until responsive)
     * 4. Add KD to dampen oscillation (start at 0.01, increase if wobbling)
     * 5. Add KI only if steady-state error (always slightly off target)
     *
     * KS = Static friction offset - voltage needed to overcome friction
     *     Start at 0, increase until wheel just starts spinning
     * KA = Acceleration compensation - helps during speed changes
     *     Usually 0.01-0.1
     */

    /** Voltage compensation */

    @JvmField
    var voltageCompEnabled = true
}


    // ==================== TURRET CONSTANTS ====================
    @Configurable
    object TurretConstants {
        /** Turret mechanical limits (degrees) */

        @JvmField
        var motorGearTeeth = 29

        @JvmField
        var outputGearTeeth = 105

        val gearRatio: Double get() = outputGearTeeth.toDouble() / motorGearTeeth.toDouble()


        /**
         * Limelight settings for turret
         * TX should be ~0 when centered on AprilTag
         * Use this tolerance to prevent oscillation
         */
        // TODO: TUNE - Adjust based on testing
        const val LL_TOLERANCE = 1.0 // degrees
        const val LL_GAIN = 0.4 // blending factor for LL + Odo
    }

    // ==================== HOOD CONSTANTS ====================
    @Configurable
    object HoodConstants {

        const val HOOD_CLOSE = 0.0  // Close range (near goal)
        const val HOOD_MID = 0.5     // Mid range
        const val HOOD_FAR = 0.7     // Far range

        /** Servo range */
        @JvmField
        var servoMinPosition = 0.0
        @JvmField
        var servoMaxPosition = 0.7

        /**
         * Distance thresholds for hood positions (meters)
         *
         */
        const val DISTANCE_CLOSE_THRESHOLD = 0.862  // < 1m = close
        const val DISTANCE_MID_THRESHOLD = 2.587   // 1-2m = mid
        // > 2m = far

        /**
         * Goal dimensions (from game manual)
         *
         */
        const val GOAL_HEIGHT = 0.984 // meters

        const val SHOOTER_HEIGHT = 0.37 // meters
    }

    // ==================== INTAKE CONSTANTS ====================
    @Configurable
    object IntakeConstants {
        /**
         * Intake motor - MEASURE THESE
         */

        const val ROLLER_DIAMETER = 0.0508 // meters

        val rollerRadius: Double get() = ROLLER_DIAMETER / 2.0

        /** Speed ratio - TUNE THROUGH TESTING */

        const val SPEED_RATIO = 2.5

        /** Power limits */
        const val INTAKE_POWER = 1.0
        const val REVERSE_POWER = -0.5
    }

    // ==================== GATE CONSTANTS ====================
    object GateConstants {
        /** Gate servo position */
        const val GATE_OPEN = 0.0
        const val GATE_CLOSED = 1.0

        /** Delay between gate open and next shot (ms) */
        const val SHOT_DELAY_MS = 500
    }

    // ==================== DRIVE CONSTANTS ====================
    @Configurable
    object DriveConstants {
        /** Starting pose */
        const val START_X = 0.0
        const val START_Y = 0.0
        const val START_HEADING = 0.0

        val startPose = Pose(START_X, START_Y, START_HEADING)

        /** Robot physical properties */

        const val WHEEL_DIAMETER = 0.104 // meters
        const val WHEEL_CIRCUMFERENCE = PI * WHEEL_DIAMETER
    }

    // ==================== SHOOTING ON THE MOVE ====================
    @Configurable
    object SOTMConstants {
        /**
         * Shooting on the move compensation
         * Uses physics from port stealing document
         *
         * Math: Adjust aim point based on robot velocity
         * Virtual goal = Actual goal + robot velocity * time_of_flight
         */
        //
        const val TIME_OF_FLIGHT = 0.8 // seconds
        // this is for far

        /** Velocity blend factor (0-1) */
        const val VELOCITY_BLEND = 0.5
    }


// Data classes for feedforward (simple version without external dependencies)
