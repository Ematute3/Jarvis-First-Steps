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
import org.firstinspires.ftc.teamcode.Shooter.Hood.Hood
import org.firstinspires.ftc.teamcode.AutoAim.AutoAim
import org.firstinspires.ftc.teamcode.Next.Shooter.FlyWheel
import org.firstinspires.ftc.teamcode.pedroPathing.Constants

@TeleOp(name = "TEST - SOTM", group = "Test")
class TestSOTM : NextFTCOpMode() {

    private var showSOTM = true

    init {
        addComponents(
            PedroComponent(Constants::createFollower),
            SubsystemComponent(Drive, FlyWheel, Hood, AutoAim),
            BulkReadComponent,
            BindingsComponent
        )
    }

    override fun onStartButtonPressed() {
        PedroDriverControlled(
            -Gamepads.gamepad1.leftStickY,
            -Gamepads.gamepad1.leftStickX,
            -Gamepads.gamepad1.rightStickX,
            false
        ).schedule()

        // Enable auto aim — periodic() handles flywheel + hood automatically
        AutoAim.setAutoAim(true)

        bindControls()
    }

    private fun bindControls() {
        Gamepads.gamepad1.a.whenBecomesTrue {
            showSOTM = !showSOTM
        }
    }

    override fun onUpdate() {

        if (showSOTM) {
            telemetry.addData("=== SOTM TEST ===", "")

            // Robot velocity
            telemetry.addData("Robot/Vel X", "%.1f".format(Drive.velocityX))
            telemetry.addData("Robot/Vel Y", "%.1f".format(Drive.velocityY))

            // Virtual goal
            telemetry.addData("Virtual/X", "%.1f".format(Drive.getVirtualGoalX()))
            telemetry.addData("Virtual/Y", "%.1f".format(Drive.getVirtualGoalY()))
            telemetry.addData("Virtual/Dist", "%.1f\"".format(Drive.distanceToVirtualGoal()))
            telemetry.addData("Virtual/Angle", "%.1f°".format(Drive.angleToVirtualGoal()))

            // AutoAim state
            telemetry.addData("Aim/Dist", "%.1f in".format(AutoAim.currentDistanceInches))
            telemetry.addData("Aim/Velocity", "%.0f t/s".format(AutoAim.targetVelocity))
            telemetry.addData("Aim/Hood", "%.2f".format(AutoAim.targetHoodPosition))

            // Comparison: static vs current
            telemetry.addData("Static/Dist", "%.1f in".format(Drive.distanceToGoal()))
        }

        // Position
        telemetry.addData("Pose/X", "%.1f".format(Drive.currentX))
        telemetry.addData("Pose/Y", "%.1f".format(Drive.currentY))

        // Flywheel status
        telemetry.addData("Flywheel/At Target", if (FlyWheel.isAtTarget()) "YES" else "NO")

        telemetry.addData("Controls:", "")
        telemetry.addData("A", "Toggle SOTM Display")
        telemetry.addData("Driving", "Move while shooting to test")
        telemetry.update()
    }
}