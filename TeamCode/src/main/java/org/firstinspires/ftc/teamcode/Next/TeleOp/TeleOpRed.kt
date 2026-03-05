package org.firstinspires.ftc.teamcode.TeleOp

import com.bylazar.telemetry.PanelsTelemetry
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

import org.firstinspires.ftc.teamcode.Lower.Drive.Drive
import org.firstinspires.ftc.teamcode.Lower.Gate.Gate
import org.firstinspires.ftc.teamcode.Lower.Intake.Intake
import org.firstinspires.ftc.teamcode.Shooter.Hood.Hood
import org.firstinspires.ftc.teamcode.Shooter.Limelight.Limelight
import org.firstinspires.ftc.teamcode.AutoAim.AutoAim
import org.firstinspires.ftc.teamcode.Next.Shooter.FlyWheel
import org.firstinspires.ftc.teamcode.Next.Shooter.Turret
import org.firstinspires.ftc.teamcode.Next.Shooter.Turret.currentState
import org.firstinspires.ftc.teamcode.Systems.Command
import org.firstinspires.ftc.teamcode.pedroPathing.Constants

@TeleOp(name = "TeleOp - Red", group = "Competition")
class TeleOpRed : NextFTCOpMode() {

    private val panelsTelemetry = PanelsTelemetry.ftcTelemetry

    init {
        addComponents(
            PedroComponent(Constants::createFollower),
            SubsystemComponent(
                Drive,
                Intake,
                Gate,
                FlyWheel,
                Turret,
                Hood,
                Limelight,
                AutoAim
            ),
            BulkReadComponent,
            BindingsComponent
        )
    }

    override fun onInit() {
        Turret.alliance = Turret.Alliance.RED
        follower.pose = Command.autoPose
    }

    override fun onStartButtonPressed() {
        PedroDriverControlled(
            -Gamepads.gamepad1.leftStickY,
            -Gamepads.gamepad1.leftStickX,
            -Gamepads.gamepad1.rightStickX,
            false
        ).schedule()
        bindControls()
    }

    // Sets follower + Drive state atomically, then re-locks turret

    private fun bindControls() {
        Gamepads.gamepad1.rightTrigger.greaterThan(0.1)
            .whenBecomesTrue(Intake.run)
            .whenBecomesFalse(Intake.stop)

        Gamepads.gamepad1.leftTrigger.greaterThan(0.1) whenBecomesTrue(Intake.reverse) whenBecomesFalse(Intake.stop)

        Gamepads.gamepad1.cross whenBecomesTrue Gate.open whenBecomesFalse Gate.close

        Gamepads.gamepad1.dpadUp.whenBecomesTrue {
            FlyWheel.setVelocity(1500.0).also({ Hood.mid() })
        }

        Gamepads.gamepad1.dpadLeft.whenBecomesTrue { Turret.nudgeLeft() }
        Gamepads.gamepad1.dpadRight.whenBecomesTrue { Turret.nudgeRight() }

        Gamepads.gamepad1.dpadDown.whenBecomesTrue {
            FlyWheel.setVelocity(1900.0).also({ Hood.far() })
        }


        // Blue side wall: x=144, y=0, heading=180
        Gamepads.gamepad1.square.whenBecomesTrue {
           // resetPose(8.0, 8.0, 180.0)
            follower.pose = Pose(0.0,8.0,180.0).also({ Turret.lock() })
        }
        Gamepads.gamepad1.triangle.whenBecomesTrue{
            Turret.runResetControl()
        }
    }

    override fun onUpdate() {
        Drive.update()
        currentState = Turret.State.LOCKED
        updateTelemetry()
    }

    override fun onStop() {
        Intake.stop
        Gate.close
        Hood.close
        FlyWheel.stop

        currentState = Turret.State.IDLE
    }

    private fun updateTelemetry() {
        telemetry.addData("=== BLUE TELEOP ===", "")
        telemetry.addData("Pose/X", "%.1f".format(Drive.currentX))
        telemetry.addData("Pose/Y", "%.1f".format(Drive.currentY))
        telemetry.addData("Pose/Heading", "%.1f°".format(Math.toDegrees(Drive.currentHeading)))
        telemetry.addData("Hood/Position", "%.2f".format(Hood.currentPosition))
        telemetry.addData("Hood/Preset", Hood.currentPreset.name)
        telemetry.addData("Zone", if (Drive.isInShootingZone()) "YES" else "NO")

        telemetry.update()
        panelsTelemetry.update()
    }
}