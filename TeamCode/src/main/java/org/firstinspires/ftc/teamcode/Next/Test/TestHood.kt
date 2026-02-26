package org.firstinspires.ftc.teamcode.Test

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import dev.nextftc.core.components.BindingsComponent
import dev.nextftc.core.components.SubsystemComponent
import dev.nextftc.ftc.Gamepads
import dev.nextftc.ftc.NextFTCOpMode
import dev.nextftc.ftc.components.BulkReadComponent
import dev.nextftc.ftc.ActiveOpMode.telemetry
import org.firstinspires.ftc.teamcode.Shooter.Hood.Hood

/**
 * Test Hood Opmode
 *
 * Tests basic hood control
 *
 * CONTROLS:
 * - D-Pad Up: Far position
 * - D-Pad Down: Close position
 * - D-Pad Left: Mid position
 * - D-Pad Right: Auto-adjust mode
 * - A/B/X/Y: Test specific servo positions
 */
@TeleOp(name = "TEST - Hood", group = "Test")
class TestHood : NextFTCOpMode() {

    init {
        addComponents(
            SubsystemComponent(Hood),
            BulkReadComponent,
            BindingsComponent
        )
    }

    override fun onStartButtonPressed() {
        bindControls()
    }

    private fun bindControls() {
        // D-Pad controls
        Gamepads.gamepad1.dpadUp.whenBecomesTrue {
            Hood.far()
            telemetry.addLine("Set to FAR")
        }

        Gamepads.gamepad1.dpadDown.whenBecomesTrue {
            Hood.close()
            telemetry.addLine("Set to CLOSE")
        }

        Gamepads.gamepad1.dpadLeft.whenBecomesTrue {
            Hood.mid()
            telemetry.addLine("Set to MID")
        }

        Gamepads.gamepad1.dpadRight.whenBecomesTrue {
            Hood.autoAdjust()
            telemetry.addLine("Set to AUTO")
        }

        // Test positions
        Gamepads.gamepad1.a.whenBecomesTrue { Hood.setPosition(0.0) }
        Gamepads.gamepad1.b.whenBecomesTrue { Hood.setPosition(0.25) } // 16.5 in
        Gamepads.gamepad1.x.whenBecomesTrue { Hood.setPosition(0.5) } // 17.25 in
        Gamepads.gamepad1.y.whenBecomesTrue { Hood.setPosition(0.70) }// 18.1

        // Back to close
        Gamepads.gamepad1.back.whenBecomesTrue { Hood.close() }
    }

    override fun onUpdate() {
        telemetry.addData("=== HOOD TEST ===", "")
        telemetry.addData("Position", "%.2f".format(Hood.currentPosition))
        telemetry.addData("Target", "%.2f".format(Hood.targetPosition))
        telemetry.addData("Preset", Hood.currentPreset.name)
        telemetry.addData("Mode", Hood.currentMode.name)

        telemetry.addData("", "")
        telemetry.addData("Controls:", "")
        telemetry.addData("DPad Up", "Far")
        telemetry.addData("DPad Down", "Close")
        telemetry.addData("DPad Left", "Mid")
        telemetry.addData("DPad Right", "Auto")
        telemetry.addData("A/B/X/Y", "Test positions")
        telemetry.update()
    }
}