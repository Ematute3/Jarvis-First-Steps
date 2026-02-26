package org.firstinspires.ftc.teamcode.ILT.Next.Subsystems.Shooter

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.util.ElapsedTime
import dev.nextftc.control.KineticState
import dev.nextftc.control.builder.controlSystem
import dev.nextftc.core.subsystems.Subsystem
import dev.nextftc.hardware.impl.MotorEx
import org.firstinspires.ftc.teamcode.FieldConstants.BLUE_GOAL_X
import org.firstinspires.ftc.teamcode.FieldConstants.GOAL_Y
import org.firstinspires.ftc.teamcode.FieldConstants.RED_GOAL_X
import org.firstinspires.ftc.teamcode.Lower.Drive.Drive
import org.firstinspires.ftc.teamcode.Lower.Drive.Drive.currentHeading
import org.firstinspires.ftc.teamcode.Lower.Drive.Drive.currentX
import org.firstinspires.ftc.teamcode.Lower.Drive.Drive.currentY
import org.firstinspires.ftc.teamcode.Lower.Drive.Drive.poseValid
import java.lang.Math.toRadians
import kotlin.math.*

object Turret : Subsystem {

    enum class State { IDLE, MANUAL, ODOMETRY, RESET_HEADING }

    var motor = MotorEx("turret")
    @JvmField var alliance = Drive.Alliance.BLUE

    var controller = controlSystem {
        posPid(0.3, 0.0, 0.04)
        basicFF(0.25, 0.0, 0.01)
    }

    var manualPower = 0.0
    var currentState = State.IDLE



    val goalX: Double
        get() = if (alliance == Drive.Alliance.RED) RED_GOAL_X else BLUE_GOAL_X
    val goalY: Double = GOAL_Y

    @JvmField var minPower: Double = 0.15
    @JvmField var maxPower: Double = 0.75

    @JvmField var kV: Double = 0.27

    const val GEAR_RATIO = 3.62068965517
    const val MOTOR_TICKS_PER_REV = 537.7
    private const val RADIANS_PER_TICK = 2.0 * PI / (MOTOR_TICKS_PER_REV * GEAR_RATIO)

    // State Tracking
    private val velTimer = ElapsedTime()
    private var lastRobotHeading = 0.0
    private var robotAngularVelocity = 0.0
    private var lastTargetSeenTime: Long = 0

    const val MIN_ANGLE = -3 * PI / 4
    const val MAX_ANGLE = 3 * PI / 4

    var turretYaw: Double = 0.0

    // Reset heading constants
    private const val RESET_TARGET_YAW = 0.0          // ← change this (e.g. Math.toRadians(90.0))
    // ~3°

    override fun initialize() {
        motor.motor.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        motor.motor.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        velTimer.reset()
        lastTargetSeenTime = System.currentTimeMillis()
    }

    override fun periodic() {
        turretYaw = getYaw()
        updateRobotVelocity()

        when (currentState) {
            State.IDLE -> {
                motor.power = manualPower.coerceIn(-maxPower, maxPower)
            }
            State.MANUAL -> {
                motor.power = manualPower.coerceIn(-maxPower, maxPower)
            }
            State.ODOMETRY -> {
                aimWithOdometryOnly()
            }
            State.RESET_HEADING -> {
                val currentYaw = getYaw()
                val error = normalizeAngle(RESET_TARGET_YAW - currentYaw)

                if (abs(error) < 0.3) {
                    motor.power = 0.0
                    currentState = State.IDLE   // or MANUAL if you want to keep control
                    return
                }

                // Use controller to drive to target (overrides ODO)
                applyControl(RESET_TARGET_YAW, 0.0)
            }
        }
    }

    private fun updateRobotVelocity() {
        // Only update when needed (in ODO or RESET)
        if (currentState != State.ODOMETRY && currentState != State.RESET_HEADING) {
            robotAngularVelocity = 0.0
            return
        }

        val dt = velTimer.seconds()
        if (dt < 0.02 || dt > 0.2) {
            robotAngularVelocity = 0.0
            velTimer.reset()
            return
        }

        val currentHeading = currentHeading
        if (currentHeading.isNaN() || currentHeading.isInfinite() || !poseValid) {
            robotAngularVelocity = 0.0
            return
        }

        val deltaHeading = normalizeAngle(currentHeading - lastRobotHeading)
        robotAngularVelocity = deltaHeading / dt
        lastRobotHeading = currentHeading
        velTimer.reset()
    }

    private fun applyControl(targetYaw: Double, targetVelocity: Double = 0.0) {
        val clampedTarget = targetYaw.coerceIn(MIN_ANGLE, MAX_ANGLE)
        val currentYaw = getYaw()

        controller.goal = KineticState(clampedTarget, targetVelocity)

        var power = controller.calculate(KineticState(currentYaw, 0.0))

        val errorDeg = Math.toDegrees(abs(clampedTarget - currentYaw))
        if (errorDeg > 0.5) {
            power += (if (power >= 0) 1.0 else -1.0) * minPower
        } else {
            if (abs(targetVelocity) < 0.1) power = 0.0
        }

        motor.power = power.coerceIn(-maxPower, maxPower)
    }

    fun aimWithOdometryOnly() {
        if (!poseValid) return
        val deltaX = goalX - currentX
        val deltaY = goalY - currentY
        val fieldAngle = atan2(deltaY, deltaX)
        val robotHeading = if (abs(currentHeading) > 2.0 * PI) Math.toRadians(currentHeading) else currentHeading
        applyControl(normalizeAngle(fieldAngle - robotHeading), -robotAngularVelocity * kV)
    }

    fun getYaw(): Double = normalizeAngle(motor.currentPosition * RADIANS_PER_TICK)

    fun normalizeAngle(radians: Double): Double {
        var angle = radians % (2.0 * PI)
        if (angle <= -PI) angle += 2.0 * PI
        if (angle > PI) angle -= 2.0 * PI
        return angle
    }

    fun aimWithOdometry() { currentState = State.ODOMETRY }

    fun stop() { currentState = State.IDLE; motor.power = 0.0 }

    fun manual() { currentState = State.MANUAL }

    // New: Trigger field-heading reset (call from TeleOp)
    fun startHeadingReset() {
        currentState = State.RESET_HEADING
        manualPower = 0.0
    }
}