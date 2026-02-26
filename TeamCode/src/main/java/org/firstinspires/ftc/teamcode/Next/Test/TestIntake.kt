package org.firstinspires.ftc.teamcode.Test

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import dev.nextftc.core.components.BindingsComponent
import dev.nextftc.core.components.SubsystemComponent
import dev.nextftc.ftc.Gamepads
import dev.nextftc.ftc.NextFTCOpMode
import dev.nextftc.ftc.components.BulkReadComponent
import dev.nextftc.ftc.ActiveOpMode.telemetry
import org.firstinspires.ftc.teamcode.Lower.Intake.Intake

/**
 * Test Intake Opmode
 *
 * Tests intake motor control
 *
 * CONTROLS:
 * - Left Trigger: Run intake
 * - Left Bumper: Reverse intake
 * - A Button: Show intake RPM calculation
 */
@TeleOp(name = "TEST - Intake", group = "Test")
class TestIntake : NextFTCOpMode() {

    init {
        addComponents(
            SubsystemComponent(Intake),
            BulkReadComponent,
            BindingsComponent
        )
    }

    override fun onStartButtonPressed() {
        bindControls()
    }

    private fun bindControls() {
        // Left Trigger: Run
        Gamepads.gamepad1.leftTrigger.greaterThan(0.5)
            .whenBecomesTrue(Intake.run)
            .whenBecomesFalse(Intake.stop)

        // Left Bumper: Reverse
        Gamepads.gamepad1.leftBumper
            .whenBecomesTrue(Intake.reverse)
            .whenBecomesFalse(Intake.stop)
    }

    override fun onUpdate() {
        telemetry.addData("=== INTAKE TEST ===", "")
        telemetry.addData("State", Intake.getState().name)

        telemetry.addData("", "")
        telemetry.addData("Controls:", "")
        telemetry.addData("L Trigger", "Run")
        telemetry.addData("L Bumper", "Reverse")
        telemetry.update()
    }
}