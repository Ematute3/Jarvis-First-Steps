package org.firstinspires.ftc.teamcode.Lower.Intake

import com.qualcomm.robotcore.hardware.DcMotorSimple
import dev.nextftc.core.commands.utility.InstantCommand
import dev.nextftc.core.subsystems.Subsystem
import dev.nextftc.ftc.ActiveOpMode
import dev.nextftc.hardware.impl.MotorEx
import org.firstinspires.ftc.teamcode.IntakeConstants

/**
 * Intake Subsystem
 * Controls roller intake for picking up game pieces
 *
 * Hardware: Motor on port "intake"
 *
 * MEASUREMENT GUIDE:
 * - ROLLER_DIAMETER: Measure roller diameter with calipers
 * - SPEED_RATIO: Test and adjust until intake speed matches drivetrain
 */
object Intake : Subsystem {
    // ==================== HARDWARE ====================
    private var intakeMotor = MotorEx("intake")

    // ==================== STATE ====================
    enum class IntakeState {
        STOPPED,
        INTAKING,
        REVERSING
    }

    var intakeState = IntakeState.STOPPED

    // ==================== INITIALIZATION ====================
    override fun initialize() {
        intakeMotor.motor.direction = DcMotorSimple.Direction.FORWARD
        stop()
    }

    // ==================== COMMANDS ====================

    /** Start intaking */
    val run = InstantCommand{
        intakeMotor.power = IntakeConstants.INTAKE_POWER
        intakeState = IntakeState.INTAKING
    }
    /** Reverse/eject */
    val reverse = InstantCommand{
        intakeMotor.power = IntakeConstants.REVERSE_POWER
        intakeState = IntakeState.REVERSING
    }

    /** Stop intake */
    val stop = InstantCommand{
        intakeMotor.power = 0.0
        intakeState = IntakeState.STOPPED
    }


    /** Get current state */
    fun getState(): IntakeState = intakeState

    // ==================== PHYSICS HELPERS ====================
    /**
     * Calculate intake RPM based on drivetrain speed
     * Uses formula: intakeRPM = drivetrainSpeed * SPEED_RATIO / rollerRadius * 60 / (2*PI)
     *
     * @param drivetrainSpeed Speed in meters/second
     * @return Required intake motor RPM
     */
    fun calculateIntakeRpm(drivetrainSpeed: Double): Double {
        val angularVelocity = (drivetrainSpeed * IntakeConstants.SPEED_RATIO) / IntakeConstants.rollerRadius
        return (60.0 * angularVelocity) / (2.0 * Math.PI)
    }

    // ==================== PERIODIC ====================
    override fun periodic() {
        // Telemetry
        ActiveOpMode.telemetry.addData("Intake/State", intakeState.name)
    }
}