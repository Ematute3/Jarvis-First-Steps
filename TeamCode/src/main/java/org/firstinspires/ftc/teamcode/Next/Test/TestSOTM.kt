package org.firstinspires.ftc.teamcode.Test

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import dev.nextftc.core.components.BindingsComponent
import dev.nextftc.core.components.SubsystemComponent
import dev.nextftc.extensions.pedro.PedroComponent
import dev.nextftc.extensions.pedro.PedroDriverControlled
import dev.nextftc.ftc.Gamepads
import dev.nextftc.ftc.NextFTCOpMode
import dev.nextftc.ftc.components.BulkReadComponent
import dev.nextftc.ftc.ActiveOpMode.telemetry
import org.firstinspires.ftc.teamcode.Lower.Drive.Drive
import org.firstinspires.ftc.teamcode.Shooter.FlyWheel.FlyWheel
import org.firstinspires.ftc.teamcode.Shooter.Hood.Hood
import org.firstinspires.ftc.teamcode.AutoAim.AutoAim
import org.firstinspires.ftc.teamcode.pedroPathing.Constants

/**
 * Test Shooting On The Move Opmode
 *
 * Tests aiming while robot is moving
 * Uses velocity to calculate virtual goal offset
 *
 * CONTROLS:
 * - Standard driving controls
 * - Right Bumper: Fire (shoots when at target)
 * - A Button: Toggle SOTM calculation display
 */
@TeleOp(name = "TEST - SOTM", group = "Test")
class TestSOTM : NextFTCOpMode() {

    private var showSOTM = true

    init {
        addComponents(
            PedroComponent(Constants::createFollower),
            SubsystemComponent(Drive, FlyWheel, Hood),
            BulkReadComponent,
            BindingsComponent
        )
    }

    override fun onStartButtonPressed() {
        // Start drivetrain
        PedroDriverControlled(
            -Gamepads.gamepad1.leftStickY,
            -Gamepads.gamepad1.leftStickX,
            -Gamepads.gamepad1.rightStickX,
            false
        ).schedule()

        // Set flywheel to auto
        FlyWheel.mid()

        bindControls()
    }

    private fun bindControls() {
        // Toggle SOTM display
        Gamepads.gamepad1.a.whenBecomesTrue {
            showSOTM = !showSOTM
        }
    }

    override fun onUpdate() {
        // Update drive
        Drive.update()

        // Calculate SOTM aim
        val aimParams = AutoAim.calculateForSOTM()

        if (showSOTM) {
            telemetry.addData("=== SOTM TEST ===", "")

            // Robot velocity
            telemetry.addData("Robot/Vel X", "%.1f".format(Drive.velocityX))
            telemetry.addData("Robot/Vel Y", "%.1f".format(Drive.velocityY))

            // Virtual goal
            telemetry.addData("", "")
            telemetry.addData("Virtual/X", "%.1f".format(Drive.getVirtualGoalX()))
            telemetry.addData("Virtual/Y", "%.1f".format(Drive.getVirtualGoalY()))
            telemetry.addData("Virtual/Dist", "%.1f\"".format(Drive.distanceToVirtualGoal()))
            telemetry.addData("Virtual/Angle", "%.1f°".format(Drive.angleToVirtualGoal()))

            // Aim params
            telemetry.addData("", "")
            telemetry.addData("Aim/Dist", "%.2fm".format(aimParams.distance))
            telemetry.addData("Aim/RPM", "%.0f".format(aimParams.rpm))
            telemetry.addData("Aim/Hood", "%.2f".format(aimParams.hoodPosition))

            // Comparison to static aim
            telemetry.addData("", "")
            telemetry.addData("Static/Dist", "%.2fm".format(Drive.distanceToGoalMeters()))
            telemetry.addData("Static/RPM", "%.0f".format(AutoAim.targetRpm))
            telemetry.addData("Diff/RPM", "%.0f".format(aimParams.rpm - AutoAim.targetRpm))
        }

        // Position
        telemetry.addData("", "")
        telemetry.addData("Pose/X", "%.1f".format(Drive.currentX))
        telemetry.addData("Pose/Y", "%.1f".format(Drive.currentY))

        // Flywheel status
        telemetry.addData("", "")
        telemetry.addData("Flywheel/Target", "%.0f".format(FlyWheel.targetVelocity))
        telemetry.addData("Flywheel/Actual", "%.0f".format(FlyWheel.getVelocity()))
        telemetry.addData("Flywheel/At Target", if (FlyWheel.isAtTarget()) "YES" else "NO")

        telemetry.addData("", "")
        telemetry.addData("Controls:", "")
        telemetry.addData("A", "Toggle SOTM Display")
        telemetry.addData("Driving", "Move while shooting to test")
    }
}