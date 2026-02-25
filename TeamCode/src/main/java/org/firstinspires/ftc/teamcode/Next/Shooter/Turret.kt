package org.firstinspires.ftc.teamcode.Shooter.Turret

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.util.ElapsedTime
import dev.nextftc.control.KineticState
import dev.nextftc.control.builder.controlSystem
import dev.nextftc.core.commands.utility.InstantCommand
import dev.nextftc.core.subsystems.Subsystem
import dev.nextftc.hardware.impl.MotorEx
import dev.nextftc.ftc.ActiveOpMode
import org.firstinspires.ftc.teamcode.Lower.Drive.Drive
import org.firstinspires.ftc.teamcode.Shooter.Limelight.Limelight
import org.firstinspires.ftc.teamcode.TurretConstants
import kotlin.math.*

/**
 * Turret Subsystem
 * Controls turret angle using odometry and optionally limelight for fine adjustment
 *
 * AIMING METHODS:
 * 1. ODO ONLY: Uses robot position and goal position to calculate angle
 *    - Formula: atan2(goalY - robotY, goalX - robotX) - robotHeading
 * 2. ODO + LL: Uses odometry for gross aim, limelight for fine adjustment
 *    - Limelight tx provides offset from centered on AprilTag
 *    - Blends odo angle with LL adjustment
 *
 * LIMELIGHT INTEGRATION:
 * - Limelight tracks AprilTags for turret alignment
 * - TX = 0 when centered on tag
 * - Use TX to fine-tune turret position
 * - Prevents oscillation with tolerance band
 *
 * MECHANICAL:
 * - Reset to 0 at start of teleop
 * - Range: -135° to +135°
 */
object Turret : Subsystem {
    // ==================== HARDWARE ====================
    lateinit var motor: MotorEx

    // ==================== CONTROLLER ====================
    var controller = controlSystem {
        posPid(
            TurretConstants.turretPosPid.kP,
            TurretConstants.turretPosPid.kI,
            TurretConstants.turretPosPid.kD
        )
        basicFF(
            TurretConstants.turretFF.kV,
            TurretConstants.turretFF.kA,
            TurretConstants.turretFF.kS
        )
    }

    // ==================== STATE ====================
    enum class AimMode {
        OFF,        // Manual control only
        ODO,        // Odometry only
        ODO_LL      // Odometry + Limelight
    }

    enum class TurretState {
        IDLE,
        MANUAL,
        AIM_ODO,
        AIM_LL
    }

    var currentState = TurretState.IDLE
    var currentAimMode = AimMode.ODO
    var manualPower = 0.0

    // ==================== ANGLE TRACKING ====================
    private val velTimer = ElapsedTime()
    private var lastHeading = 0.0
    var angularVelocity = 0.0

    // Current turret angle in radians (0 = forward)
    var turretYaw = 0.0
        private set

    // ==================== INITIALIZATION ====================
    override fun initialize() {
        motor = MotorEx("turret")
        motor.motor.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        motor.motor.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER

        // Reset to 0 at start
        turretYaw = 0.0
        lastHeading = 0.0
        velTimer.reset()
    }

    // ==================== ANGLE CONVERSIONS ====================
    /** Get current turret angle in degrees */
    val currentAngleDegrees: Double get() = Math.toDegrees(turretYaw)

    /** Convert encoder ticks to radians */
    private val TICKS_PER_RADIAN: Double get() =
        (TurretConstants.MOTOR_TICKS_PER_REV * TurretConstants.gearRatio) / (2.0 * Math.PI)

    private fun ticksToRadians(ticks: Double): Double = ticks / TICKS_PER_RADIAN
    private fun radiansToTicks(rad: Double): Int = (rad * TICKS_PER_RADIAN).toInt()

    // ==================== AIMING MODES ====================
    /**
     * Aim using odometry only
     * Calculates angle from robot position to goal
     */
    fun aimWithOdometry() {
        currentState = TurretState.AIM_ODO
    }

    /**
     * Aim using odometry + limelight
     * Odometry provides gross aim, LL fine-tunes
     */
    fun aimWithLimelight() {
        currentState = TurretState.AIM_LL
    }

    /**
     * Manual turret control
     */
    fun manual(power: Double) {
        currentState = TurretState.MANUAL
        manualPower = power
    }

    /**
     * Stop turret
     */
    val stop = InstantCommand{
        currentState = TurretState.IDLE
        motor.power = 0.0
    }

    /**
     * Reset turret to center (0 degrees)
     */
    fun reset() {
        motor.motor.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        motor.motor.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        turretYaw = 0.0
    }

    // ==================== TARGET CALCULATION ====================
    /**
     * Calculate target angle using odometry
     * @return Angle in radians
     */
    private fun calculateOdoTarget(): Double {
        if (!Drive.poseValid) return turretYaw

        val dx = Drive.goalX - Drive.currentX
        val dy = Drive.goalY - Drive.currentY
        val fieldAngle = atan2(dy, dx)

        // Convert to turret angle (robot frame)
        var targetAngle = fieldAngle - Drive.currentHeading - (Math.PI / 2) // -90 for turret offset

        return normalizeAngle(targetAngle)
    }

    /**
     * Calculate target angle using odometry + limelight
     * @return Angle in radians
     */
    private fun calculateOdoLLTarget(): Double {
        // Start with odo target
        var target = calculateOdoTarget()

        // Get limelight tx if tracking
        if (Limelight.isValidTarget()) {
            val tx = Limelight.tx

            // Check if within tolerance (prevent oscillation)
            if (abs(tx) > TurretConstants.LL_TOLERANCE) {
                // Blend LL correction with odo
                // TX is in degrees, convert to radians
                val llCorrection = Math.toRadians(tx * TurretConstants.LL_GAIN)
                target = normalizeAngle(target + llCorrection)
            }
        }

        return target
    }

    /**
     * Get angle to goal (for telemetry)
     */
    fun angleToGoal(): Double = Math.toDegrees(calculateOdoTarget())

    // ==================== CONTROL ====================
    private fun applyControl(targetYaw: Double, velocityComp: Double = 0.0) {
        // Clamp to mechanical limits
        val minRad = Math.toRadians(TurretConstants.MIN_ANGLE)
        val maxRad = Math.toRadians(TurretConstants.MAX_ANGLE)
        val clampedTarget = targetYaw.coerceIn(minRad, maxRad)

        val currentYaw = turretYaw

        // Set controller goal
        controller.goal = KineticState(clampedTarget, velocityComp)

        // Calculate power
        var power = controller.calculate(KineticState(currentYaw, 0.0))

        // Add minimum power threshold when far from target
        val errorDeg = abs(Math.toDegrees(clampedTarget - currentYaw))
        if (errorDeg > 0.5) {
            power += (if (power >= 0) 1.0 else -1.0) * TurretConstants.TURRET_MIN_POWER
        } else if (abs(velocityComp) < 0.1) {
            power = 0.0
        }

        // Clamp and apply
        motor.power = power.coerceIn(-TurretConstants.TURRET_MAX_POWER, TurretConstants.TURRET_MAX_POWER)
    }

    // ==================== VELOCITY TRACKING ====================
    private fun updateVelocity() {
        if (currentState != TurretState.AIM_ODO && currentState != TurretState.AIM_LL) {
            angularVelocity = 0.0
            return
        }

        val dt = velTimer.seconds()
        if (dt < 0.02 || dt > 0.2) {
            angularVelocity = 0.0
            velTimer.reset()
            return
        }

        val deltaHeading = normalizeAngle(Drive.currentHeading - lastHeading)
        angularVelocity = deltaHeading / dt
        lastHeading = Drive.currentHeading
        velTimer.reset()
    }

    // ==================== UTILITIES ====================
    private fun normalizeAngle(angle: Double): Double {
        var a = angle % (2.0 * Math.PI)
        if (a <= -Math.PI) a += 2.0 * Math.PI
        if (a > Math.PI) a -= 2.0 * Math.PI
        return a
    }

    // ==================== PERIODIC ====================
    override fun periodic() {
        // Update yaw from encoder
        turretYaw = ticksToRadians(motor.currentPosition.toDouble())

        // Update velocity
        updateVelocity()

        // Execute control based on state
        when (currentState) {
            TurretState.IDLE -> {
                motor.power = 0.0
            }
            TurretState.MANUAL -> {
                motor.power = manualPower.coerceIn(-TurretConstants.TURRET_MAX_POWER, TurretConstants.TURRET_MAX_POWER)
            }
            TurretState.AIM_ODO -> {
                val target = calculateOdoTarget()
                applyControl(target, -angularVelocity * 0.25)
            }
            TurretState.AIM_LL -> {
                val target = calculateOdoLLTarget()
                applyControl(target, -angularVelocity * 0.25)
            }
        }

        // Telemetry
        ActiveOpMode.telemetry.addData("Turret/Angle", "%.1f°".format(currentAngleDegrees))
        ActiveOpMode.telemetry.addData("Turret/Target", "%.1f°".format(Math.toDegrees(calculateOdoTarget())))
        ActiveOpMode.telemetry.addData("Turret/Mode", currentState.name)
    }
}