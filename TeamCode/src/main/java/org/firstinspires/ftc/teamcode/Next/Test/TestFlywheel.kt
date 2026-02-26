package org.firstinspires.ftc.teamcode.Test

import com.bylazar.telemetry.PanelsTelemetry
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import dev.nextftc.core.components.BindingsComponent
import dev.nextftc.core.components.SubsystemComponent
import dev.nextftc.ftc.Gamepads
import dev.nextftc.ftc.NextFTCOpMode
import dev.nextftc.ftc.components.BulkReadComponent
import dev.nextftc.ftc.ActiveOpMode.telemetry
import org.firstinspires.ftc.teamcode.Shooter.FlyWheel.FlyWheel
import org.firstinspires.ftc.teamcode.ShooterConstants

/**
 * Test FlyWheel Opmode
 *
 * Tests basic flywheel control without auto-adjustment
 *
 * CONTROLS:
 * - D-Pad Up: Set to far RPM (1900)
 * - D-Pad Right: Set to mid RPM (1500)
 * - D-Pad Down: Set to close RPM (1000)
 * - D-Pad Left: Turn off
 * - A Button: Toggle voltage compensation
 * - B Button: Show physics data
 */
@TeleOp(name = "TEST - Flywheel", group = "Test")
class TestFlywheel : NextFTCOpMode() {

    private var showPhysics = false

    init {
        addComponents(
            SubsystemComponent(FlyWheel),
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
            FlyWheel.setVelocity(1900.0)
            telemetry.addLine("Set to FAR (1900 RPM)")
        }

        Gamepads.gamepad1.dpadRight.whenBecomesTrue {
            FlyWheel.setVelocity(1500.0)
            telemetry.addLine("Set to MID (1500 RPM)")
        }

        Gamepads.gamepad1.dpadDown.whenBecomesTrue {
            FlyWheel.setVelocity(1000.0)
            telemetry.addLine("Set to CLOSE (1000 RPM)")
        }

        Gamepads.gamepad1.dpadLeft.whenBecomesTrue {
            FlyWheel.off()
            telemetry.addLine("Turned OFF")
        }

        // Toggle voltage compensation
        Gamepads.gamepad1.a.whenBecomesTrue {
            ShooterConstants.voltageCompEnabled = !ShooterConstants.voltageCompEnabled
        }

        // Toggle physics display
        Gamepads.gamepad1.b.whenBecomesTrue {
            showPhysics = !showPhysics
        }
    }

    override fun onUpdate() {
        telemetry.addData("=== FLYWHEEL TEST ===", "")
        telemetry.addData("Target RPM", "%.0f".format(FlyWheel.targetVelocity))
        telemetry.addData("Actual RPM", "%.0f".format(FlyWheel.getVelocity()))
        telemetry.addData("Error", "%.0f".format(FlyWheel.getError()))
        telemetry.addData("At Target", if (FlyWheel.isAtTarget()) "YES" else "NO")
        telemetry.addData("Voltage Comp", if (ShooterConstants.voltageCompEnabled) "ON" else "OFF")

        if (showPhysics) {
            telemetry.addData("Linear Vel", "%.2f m/s".format(FlyWheel.linearVelocity))
            telemetry.addData("Kinetic E", "%.2f J".format(FlyWheel.kineticEnergy))
        }

        telemetry.addData("", "")
        telemetry.addData("Controls:", "")
        telemetry.addData("DPad", "Set RPM")
        telemetry.addData("A", "Toggle Volt Comp")
        telemetry.addData("B", "Toggle Physics")
        telemetry.update()
        PanelsTelemetry.telemetry.update()
    }
}