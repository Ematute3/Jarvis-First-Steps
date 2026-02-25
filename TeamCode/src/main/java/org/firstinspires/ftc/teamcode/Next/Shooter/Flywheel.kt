package org.firstinspires.ftc.teamcode.Shooter.FlyWheel

import com.bylazar.configurables.annotations.Configurable
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.VoltageSensor
import dev.nextftc.control.ControlSystem
import dev.nextftc.control.KineticState
import dev.nextftc.control.builder.controlSystem
import dev.nextftc.core.commands.utility.InstantCommand
import dev.nextftc.core.subsystems.Subsystem
import dev.nextftc.hardware.impl.MotorEx
import dev.nextftc.ftc.ActiveOpMode
import org.firstinspires.ftc.teamcode.HoodConstants

import org.firstinspires.ftc.teamcode.ShooterConstants

/**
 * FlyWheel Subsystem
 * Controls shooter wheels with PID/FF control and voltage compensation
 *
 * TUNING GUIDE:
 *
 * 1. VOLTAGE COMPENSATION:
 *    - The code reads battery voltage and compensates motor power
 *    - This ensures consistent RPM regardless of battery level
 *    - Formula: compensatedPower = rawPower * (nominalVoltage / actualVoltage)
 *
 * 2. FEEDFORWARD (KV, KA, KS):
 *    - KV: Voltage needed per unit of velocity
 *        Formula: KV = voltage / maxRPM
 *        Start with KV = 0.001, tune until wheel reaches target speed
 *    - KA: Voltage needed for acceleration
 *        Helps during speed changes, typically 0.001-0.01
 *    - KS: Static friction offset
 *        Voltage needed to overcome friction and start wheel moving
 *        Start at 0, increase until wheel just starts spinning smoothly
 *
 * 3. PID (KP, KI, KD):
 *    - KP: Proportional - Response to current error
 *        Increase for faster response, decrease if oscillating
 *    - KI: Integral - Eliminates steady-state error
 *        Increase if always slightly below/above target
 *    - KD: Derivative - Dampens overshoot
 *        Increase if wheel wobbles or overshoots
 *
 * 4. TUNING PROCESS:
 *    a. Set all PID to 0, set KV to estimated value
 *    b. Test: Does flywheel reach target RPM?
 *    c. If never reaches target: increase KV
 *    d. If oscillates wildly: decrease KV, increase KD
 *    e. If always undershoots by fixed amount: increase KP
 *    f. If responds too slowly: increase KP
 *    g. If wobbles after reaching target: increase KD
 *    h. Only add KI if steady-state error persists
 */
object FlyWheel : Subsystem {
    // ==================== HARDWARE ====================
     var Fly1 = MotorEx("Fly1").floatMode()
     var Fly2 = MotorEx("Fly2").floatMode()
    private val battery: VoltageSensor by lazy {
        ActiveOpMode.hardwareMap.get(VoltageSensor::class.java, "Control Hub")
    }

    // ==================== CONTROLLER ====================
    private var controller: ControlSystem = buildController()

    private fun buildController(): ControlSystem = controlSystem {
        basicFF(
            ShooterConstants.ffCoefficients.kV,
            ShooterConstants.ffCoefficients.kA,
            ShooterConstants.ffCoefficients.kS
        )
        velPid(
                ShooterConstants.pidCoefficients.kP,
                ShooterConstants.pidCoefficients.kI,
                ShooterConstants.pidCoefficients.kD
        )
    }

    // ==================== VOLTAGE COMPENSATION ====================
    private var voltFiltered = ShooterConstants.NOMINAL_VOLTAGE

    /**
     * Voltage compensation implementation:
     * - Reads battery voltage each cycle
     * - Applies exponential filter to smooth readings
     * - Adjusts motor power to maintain consistent output
     *
     * Formula: adjustedPower = rawPower * (12.0 / filteredVoltage)
     * This ensures same wheel speed regardless of battery level
     */
    private fun applyVoltageCompensation(rawPower: Double): Double {
        if (!ShooterConstants.voltageCompEnabled) return rawPower

        // Read and filter voltage
        val rawVoltage = battery.voltage.coerceAtLeast(9.0) // Minimum 9V safety
        voltFiltered += ShooterConstants.VOLTAGE_FILTER_ALPHA * (rawVoltage - voltFiltered)

        // Calculate compensation ratio
        val ratio = ShooterConstants.NOMINAL_VOLTAGE / voltFiltered

        return (rawPower * ratio).coerceIn(-1.0, 1.0)
    }

    // ==================== STATE ====================
    var targetVelocity = 0.0

    // ==================== INITIALIZATION ====================
    override fun initialize() {
        // Ensure both motors spin same direction
        Fly1.motor.direction = DcMotorSimple.Direction.FORWARD
        Fly2.motor.direction = DcMotorSimple.Direction.FORWARD

        voltFiltered = ShooterConstants.NOMINAL_VOLTAGE
        targetVelocity = 0.0
        controller = buildController()
    }

    // ==================== VELOCITY CONTROL ====================
    /**
     * Set target velocity in RPM
     * @param rpm Target revolutions per minute
     */
    fun setVelocity(rpm: Double) {
        targetVelocity = rpm
        controller.goal = KineticState(0.0, rpm)
    }

    /**
     * Set velocity and rebuild controller (for updating coefficients)
     */
    fun setVelocityWithUpdate(rpm: Double) {
        targetVelocity = rpm
        controller = buildController()
        controller.goal = KineticState(0.0, rpm)
    }

    // ==================== PRESETS ====================
    /** Stop flywheel */
    val off = InstantCommand{ setVelocity(0.0)}
  val close = InstantCommand{setVelocity(ShooterConstants.FLYWHEEL_CLOSE_RPM)}
    val mid = InstantCommand{setVelocity(ShooterConstants.FLYWHEEL_MID_RPM)}
    val far = InstantCommand{setVelocity(ShooterConstants.FLYWHEEL_FAR_RPM)}


    // ==================== AUTO AIM HELPERS ====================
    /**
     * Set velocity based on distance to goal (for auto aim)
     * Linear interpolation between presets
     *
     * @param distanceMeters Distance to goal in meters
     */
    fun setVelocityFromDistance(distanceMeters: Double): Double {
        val rpm = when {
            distanceMeters < HoodConstants.DISTANCE_CLOSE_THRESHOLD ->
                ShooterConstants.FLYWHEEL_CLOSE_RPM
            distanceMeters < HoodConstants.DISTANCE_MID_THRESHOLD -> {
                // Interpolate between close and mid
                val t = (distanceMeters - HoodConstants.DISTANCE_CLOSE_THRESHOLD) /
                        (HoodConstants.DISTANCE_MID_THRESHOLD - HoodConstants.DISTANCE_CLOSE_THRESHOLD)
                ShooterConstants.FLYWHEEL_CLOSE_RPM + t *
                        (ShooterConstants.FLYWHEEL_MID_RPM - ShooterConstants.FLYWHEEL_CLOSE_RPM)
            }
            else -> {
                // Interpolate between mid and far
                val t = (distanceMeters - HoodConstants.DISTANCE_MID_THRESHOLD) / 2.0
                ShooterConstants.FLYWHEEL_MID_RPM + t.coerceAtMost(1.0) *
                        (ShooterConstants.FLYWHEEL_FAR_RPM - ShooterConstants.FLYWHEEL_MID_RPM)
            }
        }
        setVelocity(rpm)
        return rpm
    }

    // ==================== MOTOR CONTROL ====================
    private fun setMotorPowers(power: Double) {
        val clamped = power.coerceIn(-ShooterConstants.FLYWHEEL_MAX_POWER, ShooterConstants.FLYWHEEL_MAX_POWER)
        Fly1.power = clamped
        Fly2.power = clamped
    }

    // ==================== STATUS ====================
    /** Get current velocity in RPM */
    fun getVelocity(): Double = Fly1.velocity

    /** Check if at target (within tolerance) */
    fun isAtTarget(): Boolean {
        val error = targetVelocity - Fly1.velocity
        return error in ShooterConstants.VELOCITY_TOLERANCE_LOW..ShooterConstants.VELOCITY_TOLERANCE_HIGH
    }

    /** Get velocity error */
    fun getError(): Double = targetVelocity - Fly1.velocity

    // ==================== PHYSICS ====================
    /** Get linear velocity at wheel edge (m/s) */
    val linearVelocity: Double get() {
        val angularVelocity = (2.0 * Math.PI * Fly1.velocity) / 60.0
        return angularVelocity * ShooterConstants.flywheelRadius
    }

    /** Calculate kinetic energy of flywheel (Joules) */
    val kineticEnergy: Double get() {
        val angularVelocity = (2.0 * Math.PI * Fly1.velocity) / 60.0
        return 0.5 * ShooterConstants.momentOfInertia * angularVelocity * angularVelocity
    }

    // ==================== PERIODIC ====================
    override fun periodic() {
        // Calculate control output
        val rawPower = controller.calculate(Fly1.state)

        // Apply voltage compensation
        val compensatedPower = applyVoltageCompensation(rawPower)

        // Set motor powers
        setMotorPowers(compensatedPower)

        // Telemetry
        ActiveOpMode.telemetry.addData("Flywheel/Target", "%.0f RPM".format(targetVelocity))
        ActiveOpMode.telemetry.addData("Flywheel/Actual", "%.0f RPM".format(Fly1.velocity))
        ActiveOpMode.telemetry.addData("Flywheel/Error", "%.0f".format(getError()))
        ActiveOpMode.telemetry.addData("Flywheel/At Target", if (isAtTarget()) "YES" else "NO")
        ActiveOpMode.telemetry.addData("Flywheel/Power", "%.3f".format(compensatedPower))
        ActiveOpMode.telemetry.addData("Flywheel/Voltage", "%.1fV".format(voltFiltered))
    }
}