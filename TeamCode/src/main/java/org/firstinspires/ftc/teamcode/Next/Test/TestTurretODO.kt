package org.firstinspires.ftc.teamcode.Test

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import dev.nextftc.core.components.BindingsComponent
import dev.nextftc.core.components.SubsystemComponent
import dev.nextftc.ftc.Gamepads
import dev.nextftc.ftc.NextFTCOpMode
import dev.nextftc.ftc.components.BulkReadComponent
import dev.nextftc.ftc.ActiveOpMode.telemetry
import org.firstinspires.ftc.teamcode.Lower.Drive.Drive
import org.firstinspires.ftc.teamcode.Shooter.Turret.Turret
import org.firstinspires.ftc.teamcode.Shooter.Limelight.Limelight

/**
 * Test Turret ODO Opmode
 *
 * Tests turret aiming with odometry only
 *
 * CONTROLS:
 * - Left Stick X: Manual turret control
 * - A Button: Enable odometry aiming
 * - B Button: Reset turret
 * - X Button: Stop turret
 */
@TeleOp(name = "TEST - Turret ODO", group = "Test")
class TestTurretODO : NextFTCOpMode() {

    init {
        addComponents(
            SubsystemComponent(Drive, Turret, Limelight),
            BulkReadComponent,
            BindingsComponent
        )
    }

    override fun onStartButtonPressed() {
        bindControls()
    }

    private fun bindControls() {
        // Left Stick: Manual control

        // A: Enable odometry aiming
        Gamepads.gamepad1.a.whenBecomesTrue {
            Turret.aimWithOdometry()
        }

        // B: Reset turret
        Gamepads.gamepad1.b.whenBecomesTrue {
            Turret.reset()
        }

        // X: Stop
        Gamepads.gamepad1.x.whenBecomesTrue {
            Turret.stop()
        }
    }

    override fun onUpdate() {
        // Update drive
        Drive.update()
        Limelight.update()

        telemetry.addData("=== TURRET ODO TEST ===", "")
        telemetry.addData("State", Turret.currentState.name)
        telemetry.addData("Angle", "%.1f°".format(Turret.currentAngleDegrees))
        telemetry.addData("Target", "%.1f°".format(Turret.angleToGoal()))

        // Show position
        telemetry.addData("", "")
        telemetry.addData("Robot/X", "%.1f".format(Drive.currentX))
        telemetry.addData("Robot/Y", "%.1f".format(Drive.currentY))
        telemetry.addData("Robot/Heading", "%.1f°".format(Math.toDegrees(Drive.currentHeading)))
        telemetry.addData("Dist to Goal", "%.1f\"".format(Drive.distanceToGoal()))

        // Show zone
        telemetry.addData("In Zone", if (Drive.isInShootingZone()) "YES" else "NO")

        telemetry.addData("", "")
        telemetry.addData("Controls:", "")
        telemetry.addData("L Stick", "Manual")
        telemetry.addData("A", "Aim ODO")
        telemetry.addData("B", "Reset")
        telemetry.addData("X", "Stop")
    }
}