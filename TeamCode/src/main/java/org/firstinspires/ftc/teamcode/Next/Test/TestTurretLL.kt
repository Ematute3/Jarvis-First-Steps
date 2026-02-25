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
 * Test Turret LL Opmode
 *
 * Tests turret aiming with odometry + limelight
 *
 * CONTROLS:
 * - A Button: Enable ODO + LL aiming
 * - B Button: Reset turret
 * - X Button: Stop turret
 */
@TeleOp(name = "TEST - Turret LL", group = "Test")
class TestTurretLL : NextFTCOpMode() {

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
        // A: Enable odometry + limelight aiming
        Gamepads.gamepad1.a.whenBecomesTrue {
            Turret.aimWithLimelight()
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

        telemetry.addData("=== TURRET LL TEST ===", "")
        telemetry.addData("State", Turret.currentState.name)
        telemetry.addData("Angle", "%.1f°".format(Turret.currentAngleDegrees))
        telemetry.addData("Target", "%.1f°".format(Turret.angleToGoal()))

        // Limelight data
        telemetry.addData("", "")
        telemetry.addData("LL/Tracking", if (Limelight.isTracking) "YES" else "NO")
        telemetry.addData("LL/TX", "%.1f".format(Limelight.tx))
        telemetry.addData("LL/TY", "%.1f".format(Limelight.ty))
        telemetry.addData("LL/TA", "%.1f".format(Limelight.ta))

        telemetry.addData("", "")
        telemetry.addData("Controls:", "")
        telemetry.addData("A", "Aim ODO+LL")
        telemetry.addData("B", "Reset")
        telemetry.addData("X", "Stop")

        Limelight.updateTelemetry()
    }
}