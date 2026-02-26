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

// Angle limits (degrees) - physical constraints
const val TURRET_MIN_ANGLE = -135.0
const val TURRET_MAX_ANGLE = 135.0

// Power limits - tunable
const val TURRET_MIN_POWER = 0.15  // Minimum power to overcome friction
const val TURRET_MAX_POWER = 1.0   // Maximum motor power

object Turret : Subsystem {

    enum class State {
        IDLE,
        MANUAL,
        LOCKED,
        RESET
    }

    var motor = MotorEx("turret")

    var currentState = State.IDLE
    var manualPower = 0.0

    var targetYaw = 0.0
    var isLocked = false

    // Velocity tracking
    private val velTimer = ElapsedTime()
    private var lastRobotHeading = 0.0
    var robotAngularVelocity = 0.0

    @JvmField var alliance = Drive.Alliance.BLUE

    private val goalX: Double get() = if (alliance == Drive.Alliance.RED) RED_GOAL_X else BLUE_GOAL_X
    private val goalY: Double = GOAL_Y

    private var controller = buildController()

    private fun buildController() = controlSystem {
        posPid(turretPosPid.kP, turretPosPid.kI, turretPosPid.kD)
        basicFF(turretFF.kV, turretFF.kA, turretFF.kS)
    }

    // ==================== INITIALIZATION ====================
    override fun initialize() {
        motor.motor.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        motor.motor.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER

        velTimer.reset()
        lastRobotHeading = 0.0
        robotAngularVelocity = 0.0

        controller = buildController()

        targetYaw = getYaw()
    }

    // ==================== SOFT START (commented out) ====================
    // Uncomment and call applySoftStart() if turret jerks on initial lock-on
    /*
    private var softStartRamping = false
    private fun applySoftStart(targetPower: Double): Double {
        // Implementation: ramp power gradually from current to target
        // Use a ramp rate constant to control speed
        return targetPower
    }
    */

    // ==================== PERIODIC ====================
    override fun periodic() {
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

        ActiveOpMode.telemetry.addData("Turret/State", currentState.name)
        ActiveOpMode.telemetry.addData("Turret/Yaw", "%.2f°".format(Math.toDegrees(getYaw())))
        ActiveOpMode.telemetry.addData("Turret/Target", "%.2f°".format(Math.toDegrees(targetYaw)))
        ActiveOpMode.telemetry.addData("Turret/Locked", if (isLocked) "YES" else "NO")
        ActiveOpMode.telemetry.addData("Turret/RobotVel", "%.2f°/s".format(Math.toDegrees(robotAngularVelocity)))
    }

    // ==================== LOCKED CONTROL ====================
    private fun runLockedControl() {
        if (Drive.poseValid) {
            val deltaX = goalX - Drive.currentX
            val deltaY = goalY - Drive.currentY
            val fieldAngleToGoal = atan2(deltaY, deltaX)

            val robotHeadingRad = Drive.currentHeading
            targetYaw = normalizeAngle(fieldAngleToGoal - robotHeadingRad)
        }

        // Clamp target to physical limits
        targetYaw = targetYaw.coerceIn(
            Math.toRadians(TURRET_MIN_ANGLE),
            Math.toRadians(TURRET_MAX_ANGLE)
        )

        val currentYaw = getYaw()
        val error = normalizeAngle(targetYaw - currentYaw)
        val errorDeg = Math.toDegrees(abs(error))

        isLocked = errorDeg < ALIGNMENT_TOLERANCE

        val rotationCompensation = -robotAngularVelocity * 1.5

        controller.goal = KineticState(targetYaw, rotationCompensation)
        var power = controller.calculate(KineticState(currentYaw, robotAngularVelocity))

        // Apply minimum power threshold to overcome friction
        if (errorDeg > ALIGNMENT_TOLERANCE) {
            val direction = sign(error)
            power += direction * TURRET_MIN_POWER
        } else {
            // When locked, can relax
            if (abs(power) < TURRET_MIN_POWER * 0.5) {
                power = 0.0
            }
        }

        // Clamp final power to limits
        motor.power = power.coerceIn(-TURRET_MAX_POWER, TURRET_MAX_POWER)
    }

    // ==================== RESET CONTROL ====================
    private fun runResetControl() {
        val currentYaw = getYaw()
        val error = normalizeAngle(0.0 - currentYaw)
        val errorDeg = Math.toDegrees(abs(error))

        if (errorDeg < ALIGNMENT_TOLERANCE) {
            motor.power = 0.0
            targetYaw = 0.0
            currentState = State.IDLE
            return
        }

        controller.goal = KineticState(0.0, 0.0)
        var power = controller.calculate(KineticState(currentYaw, 0.0))

        if (errorDeg > ALIGNMENT_TOLERANCE) {
            power += sign(error) * TURRET_MIN_POWER
        }

        motor.power = power.coerceIn(-TURRET_MAX_POWER, TURRET_MAX_POWER)
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


    fun stop() {
        currentState = State.IDLE
        motor.power = 0.0
    }

    fun setManual(power: Double) {
        currentState = State.MANUAL
        manualPower = power
    }

    fun resetToCenter() {
        currentState = State.RESET
    }

    fun getYawDegrees(): Double = Math.toDegrees(getYaw())

    fun getYaw(): Double {
        val ticks = motor.currentPosition
        return ticksToRadians(ticks)
    }

    private fun ticksToRadians(ticks: Double): Double {
        return ticks * TurretConstants.degreesPerTick * (Math.PI / 180.0)
    }

    fun normalizeAngle(radians: Double): Double {
        var angle = radians % (2.0 * PI)
        if (angle <= -PI) angle += 2.0 * PI
        if (angle > PI) angle -= 2.0 * PI
        return angle
    }

    fun setAlliance(red: Boolean) {
        alliance = if (red) Drive.Alliance.RED else Drive.Alliance.BLUE
    }

    fun rebuildController() {
        controller = buildController()
    }
}