package org.firstinspires.ftc.teamcode.Test

import com.pedropathing.geometry.Pose
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import dev.nextftc.core.components.BindingsComponent
import dev.nextftc.core.components.SubsystemComponent
import dev.nextftc.extensions.pedro.PedroComponent
import dev.nextftc.extensions.pedro.PedroComponent.Companion.follower
import dev.nextftc.extensions.pedro.PedroDriverControlled
import dev.nextftc.ftc.Gamepads
import dev.nextftc.ftc.NextFTCOpMode
import dev.nextftc.ftc.components.BulkReadComponent
import dev.nextftc.ftc.ActiveOpMode.telemetry
import org.firstinspires.ftc.teamcode.Lower.Drive.Drive
import org.firstinspires.ftc.teamcode.Next.Shooter.Turret
import org.firstinspires.ftc.teamcode.Shooter.Limelight.Limelight
import org.firstinspires.ftc.teamcode.pedroPathing.Constants
import org.firstinspires.ftc.teamcode.pedroPathing.Constants.createFollower

/**
 * Test Turret ODO OpMode
 *
 * Tests turret aiming with odometry only
 *
 * CONTROLS:
 * - Left Stick X: Manual turret control
 * - A Button: Enable odometry aiming (lock on)
 * - B Button: Reset turret to center
 * - X Button: Stop turret
 * - D-Pad Left/Right: Switch alliance (Red/Blue)
 */

@TeleOp(name = "TEST - Turret", group = "Test")
class TestTurret : NextFTCOpMode() {

    init {
        addComponents(
            PedroComponent(Constants::createFollower),
            SubsystemComponent(Drive, Turret, Limelight),
            BulkReadComponent,
            BindingsComponent
        )
    }

    override fun onInit() {
        // Start in corner for testing
        follower.pose = Pose(72.0, 72.0, 0.0)
    }

    override fun onStartButtonPressed() {
        // Start drivetrain
        PedroDriverControlled(
            -Gamepads.gamepad1.leftStickY,
            -Gamepads.gamepad1.leftStickX,
            -Gamepads.gamepad1.rightStickX,
            false
        ).schedule()

        bindControls()
    }

    private fun bindControls() {
        // A: Lock on (aim with odometry)
        Gamepads.gamepad1.a.whenBecomesTrue {
            Turret.lockOn()
        }

        // B: Reset to center
        Gamepads.gamepad1.b.whenBecomesTrue {
            Turret.resetToCenter()
        }

        // X: Stop
        Gamepads.gamepad1.x.whenBecomesTrue {
            Turret.stop()
        }

        // Y: Manual control
        Gamepads.gamepad1.y.whenBecomesTrue {
            Turret.setManual(0.0) // Will be controlled by stick
        }

        // D-Pad Left: Manual left
        Gamepads.gamepad1.dpadLeft.whenBecomesTrue{
            Turret.setManual(-0.5)
        }

        // D-Pad Right: Manual right
        Gamepads.gamepad1.dpadRight.whenBecomesTrue {
            Turret.setManual(0.5)
        }
    }

    override fun onUpdate() {
        // Update drive
        Drive.update()
        Limelight.update()
        Turret.periodic()

        telemetry.addData("=== TURRET TEST ===", "")
        telemetry.addData("State", Turret.currentState.name)

        // Turret position
        telemetry.addData("", "")
        telemetry.addData("Turret/Yaw", "%.2f°".format(Math.toDegrees(Turret.getYaw())))
        telemetry.addData("Turret/Target", "%.2f°".format(Math.toDegrees(Turret.targetYaw)))

        // Calculate error
        val error = kotlin.math.abs(Turret.normalizeAngle(Turret.targetYaw - Turret.getYaw()))
        telemetry.addData("Turret/Error", "%.2f°".format(Math.toDegrees(error)))
        telemetry.addData("Turret/Locked", if (Turret.isLocked) "YES" else "NO")

        // Robot position
        telemetry.addData("", "")
        telemetry.addData("Robot/X", "%.1f".format(Drive.currentX))
        telemetry.addData("Robot/Y", "%.1f".format(Drive.currentY))
        telemetry.addData("Robot/Heading", "%.1f°".format(Math.toDegrees(Drive.currentHeading)))
        telemetry.addData("Robot/Vel", "%.1f°/s".format(Math.toDegrees(Turret.robotAngularVelocity)))

        // Goal info
        telemetry.addData("", "")
        telemetry.addData("Dist to Goal", "%.1f\"".format(Drive.distanceToGoal()))
        telemetry.addData("Angle to Goal", "%.1f°".format(Drive.angleToGoal()))
        telemetry.addData("Alliance", if (Turret.alliance == Drive.Alliance.RED) "RED" else "BLUE")

        // Zone
        telemetry.addData("In Zone", if (Drive.isInShootingZone()) "YES" else "NO")

        telemetry.addData("", "")
        telemetry.addData("Controls:", "")
        telemetry.addData("Left Stick X", "Manual turret")
        telemetry.addData("A / D-Pad Up", "Lock On")
        telemetry.addData("B / D-Pad Down", "Reset")
        telemetry.addData("X", "Stop")
        telemetry.addData("D-Pad L/R", "Manual rotate")

        telemetry.update()
    }
}