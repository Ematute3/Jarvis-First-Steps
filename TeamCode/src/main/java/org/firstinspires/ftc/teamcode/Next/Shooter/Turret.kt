package org.firstinspires.ftc.teamcode.Next.Shooter

import com.bylazar.configurables.annotations.Configurable
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
import org.firstinspires.ftc.teamcode.ShooterConstants.TurretConstants.motorGearTeeth
import org.firstinspires.ftc.teamcode.ShooterConstants.TurretConstants.outputGearTeeth
import java.lang.Math.toRadians
import kotlin.math.*

// Angle limits (degrees) - physical constraints
const val TURRET_MIN_ANGLE = -135.0
const val TURRET_MAX_ANGLE = 135.0

// East Loop ELC Encoder V2 - Quadrature (4000 CPR)
const val ENCODER_CPR = 4000  // Counts per revolution

@Configurable
object Turret : Subsystem {

    enum class State {
        IDLE,
        MANUAL,
        LOCKED,
        RESET
    }

    // ==================== TUNABLE COEFFICIENTS ====================
    @JvmField var kP: Double = 0.15
    @JvmField var kI: Double = 0.0
    @JvmField var kD: Double = 0.01

    @JvmField var ffKV: Double = 0.32
    @JvmField var ffKA: Double = 0.0
    @JvmField var ffKS: Double = 0.01

    // Other tunables
    @JvmField var minPower: Double = 0.15
    @JvmField var maxPower: Double = 0.75
    @JvmField var alignmentTolerance: Double = 2.0
    @JvmField var rotationCompGain: Double = 1.5

    // Motor - encoder plugged into motor's encoder port
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
    private val goalY: Double get() = GOAL_Y

    private var controller = buildController()

    private fun buildController() = controlSystem {
        posPid(kP, kI, kD)
        basicFF(ffKV, ffKA, ffKS)
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

    // ==================== PERIODIC ====================
    override fun periodic() {
        // Rebuild controller each cycle to pick up live Configurable changes
        controller = buildController()

        if (currentState == State.LOCKED) {
            updateRobotAngularVelocity()
        }

        when (currentState) {
            State.IDLE -> {
                motor.power = 0.0
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
    fun runLockedControl() {
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
        val rotationCompensation = -robotAngularVelocity * rotationCompGain

        controller.goal = KineticState(targetYaw, rotationCompensation)
        val power = controller.calculate(KineticState(currentYaw, robotAngularVelocity))

        // Apply minimum power threshold to overcome friction


        motor.power = power.coerceIn(-maxPower, maxPower)
    }

    // ==================== RESET CONTROL ====================
    private fun runResetControl() {
        val currentYaw = getYaw()
        val error = normalizeAngle(0.0 - currentYaw)
        val errorDeg = Math.toDegrees(abs(error))

        if (errorDeg < alignmentTolerance) {
            motor.power = 0.0
            targetYaw = 0.0
            currentState = State.IDLE
            return
        }

        controller.goal = KineticState(0.0, 0.0)
        var power = controller.calculate(KineticState(currentYaw, 0.0))

        if (errorDeg > alignmentTolerance) {
            power += sign(error) * minPower
        }

        motor.power = power.coerceIn(-maxPower, maxPower)
    }

    // ==================== VELOCITY TRACKING ====================
    private fun updateRobotAngularVelocity() {
        if (!Drive.poseValid) {
            robotAngularVelocity = 0.0
            return
        }

        val dt = velTimer.seconds()
        if (dt !in 0.01..0.2) {
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
        val ticks = motor.currentPosition.toDouble()
        return ticksToRadians(ticks)
    }

    private fun ticksToRadians(ticks: Double): Double {
        val gearRatio = motorGearTeeth.toDouble() / outputGearTeeth.toDouble()
        return ticks * (2.0 * PI / ENCODER_CPR) * gearRatio
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

    fun reZeroEncoder() {
        motor.motor.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        motor.motor.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
    }
}