package org.firstinspires.ftc.teamcode.Next.Shooter

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.util.ElapsedTime
import dev.nextftc.control.KineticState
import dev.nextftc.control.builder.controlSystem
import dev.nextftc.core.subsystems.Subsystem
import dev.nextftc.hardware.impl.MotorEx
import dev.nextftc.ftc.ActiveOpMode
import org.firstinspires.ftc.teamcode.FieldConstants.BLUE_GOAL_X
import org.firstinspires.ftc.teamcode.FieldConstants.GOAL_Y
import org.firstinspires.ftc.teamcode.FieldConstants.RED_GOAL_X
import org.firstinspires.ftc.teamcode.Lower.Drive.Drive

import org.firstinspires.ftc.teamcode.TurretConstants
import org.firstinspires.ftc.teamcode.TurretConstants.ALIGNMENT_TOLERANCE
import org.firstinspires.ftc.teamcode.TurretConstants.turretFF
import org.firstinspires.ftc.teamcode.TurretConstants.turretPosPid
import java.lang.Math.toRadians
import kotlin.math.*

object Turret : Subsystem {

    enum class State {
        IDLE,          // No automatic control
        MANUAL,        // Driver controls directly
        LOCKED,        // Locked onto target, actively maintains position
        RESET          // Resetting to center
    }

    // Hardware
    var motor = MotorEx("turret")

    // State
    var currentState = State.IDLE
    var manualPower = 0.0

    // Target tracking
    var targetYaw = 0.0              // Where we want turret to point (world-relative radians)
    var isLocked = false             // Are we on target?

    // Velocity tracking
    private val velTimer = ElapsedTime()
    private var lastRobotHeading = 0.0
    var robotAngularVelocity = 0.0

    // Alliance
    @JvmField var alliance = Drive.Alliance.BLUE

    // Goal position
    private val goalX: Double
        get() = if (alliance == Drive.Alliance.RED) RED_GOAL_X else BLUE_GOAL_X
    private val goalY: Double = GOAL_Y

    // Controller - rebuilt when constants change
    private var controller = buildController()

    private fun buildController() = controlSystem {
        posPid(turretPosPid.kP, turretPosPid.kI, turretPosPid.kD)
        basicFF(turretFF.kV, turretFF.kA, turretFF.kS)
    }

    // ==================== INITIALIZATION ====================
    override fun initialize() {
        motor.motor.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        motor.motor.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER

        // Reset velocity tracking
        velTimer.reset()
        lastRobotHeading = 0.0
        robotAngularVelocity = 0.0

        // Build fresh controller
        controller = buildController()

        // Start at current position
        targetYaw = getYaw()
    }

    // ==================== PERIODIC ====================
    override fun periodic() {
        // Always track robot angular velocity when locked
        if (currentState == State.LOCKED) {
            updateRobotAngularVelocity()
        }

        when (currentState) {
            State.IDLE -> {
                motor.power = manualPower.coerceIn(-1.0, 1.0)
            }

            State.MANUAL -> {
                motor.power = manualPower.coerceIn(-1.0, 1.0)
            }

            State.LOCKED -> {
                runLockedControl()
            }

            State.RESET -> {
                runResetControl()
            }
        }

        // Telemetry
        ActiveOpMode.telemetry.addData("Turret/State", currentState.name)
        ActiveOpMode.telemetry.addData("Turret/Yaw", "%.2f°".format(Math.toDegrees(getYaw())))
        ActiveOpMode.telemetry.addData("Turret/Target", "%.2f°".format(Math.toDegrees(targetYaw)))
        ActiveOpMode.telemetry.addData("Turret/Locked", if (isLocked) "YES" else "NO")
        ActiveOpMode.telemetry.addData("Turret/RobotVel", "%.2f°/s".format(Math.toDegrees(robotAngularVelocity)))
    }

    // ==================== LOCKED CONTROL ====================
    private fun runLockedControl() {
        // Calculate target angle to goal (world-relative)
        if (Drive.poseValid) {
            val deltaX = goalX - Drive.currentX
            val deltaY = goalY - Drive.currentY
            val fieldAngleToGoal = atan2(deltaY, deltaX)  // World-relative angle to goal

            // Target yaw = where turret needs to point in world to hit goal
            // Turret angle = field angle - robot heading (convert robot heading to radians)
            val robotHeadingRad = Drive.currentHeading
            targetYaw = normalizeAngle(fieldAngleToGoal - robotHeadingRad)
        }

        val currentYaw = getYaw()
        val error = normalizeAngle(targetYaw - currentYaw)
        val errorDeg = Math.toDegrees(abs(error))

        // Check if locked on target
        isLocked = errorDeg < ALIGNMENT_TOLERANCE

        // COMPENSATE FOR ROBOT ROTATION
        // If robot spins clockwise, turret must spin counter-clockwise to maintain world-relative aim
        // The compensation should equal negative of robot angular velocity
        val rotationCompensation = -robotAngularVelocity * 1.5  // 1.5x for extra responsiveness

        // Run PID controller
        controller.goal = KineticState(targetYaw, rotationCompensation)
        var power = controller.calculate(KineticState(currentYaw, robotAngularVelocity))

        // Add minimum power threshold to overcome friction when far from target
        if (errorDeg > ALIGNMENT_TOLERANCE) {
            val direction = sign(error)
            power += direction * 0.15
        } else {
            // When locked, still apply small holding power
            if (abs(power) < 0.15 * 0.5) {
                power = 0.0  // Actually at target, can relax
            }
        }

        motor.power = power.coerceIn(-1.0, 1.0)
    }

    // ==================== RESET CONTROL ====================
    private fun runResetControl() {
        val currentYaw = getYaw()
        val error = normalizeAngle(0.0 - currentYaw)  // Target = 0
        val errorDeg = Math.toDegrees(abs(error))

        if (errorDeg < ALIGNMENT_TOLERANCE) {
            // Reached center
            motor.power = 0.0
            targetYaw = 0.0
            currentState = State.IDLE
            return
        }

        // Use controller for reset too
        controller.goal = KineticState(0.0, 0.0)
        var power = controller.calculate(KineticState(currentYaw, 0.0))

        // Add min power
        if (errorDeg > ALIGNMENT_TOLERANCE) {
            power += sign(error) * 0.15
        }

        motor.power = power.coerceIn(-1.0, 1.0)
    }

    // ==================== VELOCITY TRACKING ====================
    private fun updateRobotAngularVelocity() {
        if (!Drive.poseValid) {
            robotAngularVelocity = 0.0
            return
        }

        val dt = velTimer.seconds()
        if (dt < 0.01 || dt > 0.2) {
            velTimer.reset()
            return
        }

        val currentHeading = Drive.currentHeading

        // Handle NaN/Infinity
        if (currentHeading.isNaN() || currentHeading.isInfinite()) {
            velTimer.reset()
            return
        }

        val deltaHeading = normalizeAngle(currentHeading - lastRobotHeading)
        robotAngularVelocity = deltaHeading / dt

        lastRobotHeading = currentHeading
        velTimer.reset()
    }

    // ==================== PUBLIC METHODS ====================

    /**
     * Lock turret onto goal - call this to start auto-aiming
     * Turret will quickly snap to target and stay locked
     */
    fun lockOn() {
        currentState = State.LOCKED
        // Recalculate target immediately
        if (Drive.poseValid) {
            val deltaX = goalX - Drive.currentX
            val deltaY = goalY - Drive.currentY
            val fieldAngleToGoal = atan2(deltaY, deltaX)
            val robotHeadingRad = Drive.currentHeading
            targetYaw = normalizeAngle(fieldAngleToGoal - robotHeadingRad)
        }
    }

    /**
     * Stop auto-aim, return to manual control
     */
    fun stop() {
        currentState = State.IDLE
        motor.power = 0.0
    }

    /**
     * Manual control mode - driver has direct control
     */
    fun setManual(power: Double) {
        currentState = State.MANUAL
        manualPower = power
    }

    /**
     * Reset turret to center (0 degrees)
     */
    fun resetToCenter() {
        currentState = State.RESET
    }

    /**
     * Get current turret angle in degrees
     */
    fun getYawDegrees(): Double = Math.toDegrees(getYaw())

    /**
     * Get current turret angle in radians
     */
    fun getYaw(): Double {
        val ticks = motor.currentPosition
        return ticksToRadians(ticks)
    }

    /**
     * Convert motor ticks to radians
     */
    private fun ticksToRadians(ticks: Double): Double {
        return ticks * TurretConstants.degreesPerTick * (Math.PI / 180.0)
    }

    /**
     * Normalize angle to -PI to +PI range
     */
    fun normalizeAngle(radians: Double): Double {
        var angle = radians % (2.0 * PI)
        if (angle <= -PI) angle += 2.0 * PI
        if (angle > PI) angle -= 2.0 * PI
        return angle
    }

    /**
     * Set alliance (call when game starts)
     */
    fun setAlliance(red: Boolean) {
        alliance = if (red) Drive.Alliance.RED else Drive.Alliance.BLUE
    }

    /**
     * Rebuild controller (call after changing PID constants)
     */
    fun rebuildController() {
        controller = buildController()
    }
}