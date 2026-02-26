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

import org.firstinspires.ftc.teamcode.FieldConstants
import org.firstinspires.ftc.teamcode.Lower.Drive.Drive
import org.firstinspires.ftc.teamcode.pedroPathing.Constants

/**
 * Test Drive Opmode
 *
 * Tests drive with full telemetry
 * Shows position, velocity, shooting zone status
 *
 * CONTROLS:
 * - Standard field-centric driving
 */
@TeleOp(name = "TEST - Drive", group = "Test")
class TestDrive : NextFTCOpMode() {

    init {
        addComponents(
            PedroComponent(Constants::createFollower),
            SubsystemComponent(Drive),
            BulkReadComponent,
            BindingsComponent
        )
    }

    override fun onInit() {
       // Drive.lastKnown = Pose(72.0, 72.0, 0.0)
        follower.pose = Pose(72.0,72.0,0.0)
    }

    override fun onStartButtonPressed() {
        // Start drivetrain
        PedroDriverControlled(
            -Gamepads.gamepad1.leftStickY,
            -Gamepads.gamepad1.leftStickX,
            -Gamepads.gamepad1.rightStickX,
            false
        ).schedule()
    }

    override fun onUpdate() {
        // Update drive
        Drive.update()

        telemetry.addData("=== DRIVE TEST ===", "")

        // Position
        telemetry.addData("Pose/Valid", if (Drive.poseValid) "YES" else "NO")
        if (Drive.poseValid) {
            telemetry.addData("Pose/X", "%.1f".format(Drive.currentX))
            telemetry.addData("Pose/Y", "%.1f".format(Drive.currentY))
            telemetry.addData("Pose/Heading", "%.1f°".format(Math.toDegrees(Drive.currentHeading)))
        }

        // Velocity
        telemetry.addData("", "")
        telemetry.addData("Velocity/X", "%.1f".format(Drive.velocityX))
        telemetry.addData("Velocity/Y", "%.1f".format(Drive.velocityY))

        // Goal distance
        telemetry.addData("", "")
        telemetry.addData("Goal/Dist", "%.1f\"".format(Drive.distanceToGoal()))
        telemetry.addData("Goal/Angle", "%.1f°".format(Drive.angleToGoal()))

        // Shooting zones
        telemetry.addData("", "")
        telemetry.addData("Zone/Main", if (FieldConstants.isInMainShootingZone(Drive.currentX, Drive.currentY)) "YES" else "NO")
        telemetry.addData("Zone/Secondary", if (FieldConstants.isInSecondaryShootingZone(Drive.currentX, Drive.currentY)) "YES" else "NO")
        telemetry.addData("Zone/Any", if (Drive.isInAnyShootingZone()) "YES" else "NO")

        // Alliance
        telemetry.addData("", "")
        telemetry.addData("Alliance", Drive.alliance.name)
        telemetry.update()
    }
}