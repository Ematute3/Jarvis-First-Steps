package org.firstinspires.ftc.teamcode.Autonomoous.Red

import com.bylazar.configurables.annotations.Configurable
import com.pedropathing.geometry.BezierCurve
import com.pedropathing.geometry.BezierLine
import com.pedropathing.geometry.Pose
import com.pedropathing.paths.PathChain
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import dev.nextftc.core.commands.delays.Delay
import dev.nextftc.core.commands.groups.SequentialGroup
import dev.nextftc.core.components.BindingsComponent
import dev.nextftc.core.components.SubsystemComponent
import dev.nextftc.extensions.pedro.FollowPath
import dev.nextftc.extensions.pedro.PedroComponent
import dev.nextftc.extensions.pedro.PedroComponent.Companion.follower
import dev.nextftc.ftc.NextFTCOpMode
import dev.nextftc.ftc.components.BulkReadComponent
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.Lower.Drive.Drive
import org.firstinspires.ftc.teamcode.Lower.Gate.Gate
import org.firstinspires.ftc.teamcode.Lower.Intake.Intake
import org.firstinspires.ftc.teamcode.Next.Shooter.FlyWheel
import org.firstinspires.ftc.teamcode.Next.Shooter.Turret
import org.firstinspires.ftc.teamcode.Shooter.Hood.Hood
import org.firstinspires.ftc.teamcode.Systems.Command
import org.firstinspires.ftc.teamcode.Systems.ShooterCommands
import org.firstinspires.ftc.teamcode.Systems.initCommands
import org.firstinspires.ftc.teamcode.Systems.intakeAuto
import org.firstinspires.ftc.teamcode.pedroPathing.Constants

@Configurable
@Autonomous(name = "12-blue-far", group = "Competition", preselectTeleOp = "TeleOp - Blue")
class AutoBlue21 : NextFTCOpMode() {

    init {
        addComponents(
            PedroComponent(Constants::createFollower),
            SubsystemComponent(
                Drive, Intake, Gate, FlyWheel, Turret, Hood
            ),
            BulkReadComponent, BindingsComponent
        )
    }

    private var paths: Array<PathChain> = arrayOf()

    override fun onInit() {
        Turret.alliance = Turret.Alliance.BLUE
    }

    override fun onStartButtonPressed() {
        follower.setStartingPose(Pose(56.000, 9.000, Math.toRadians(180.0)))
        buildPaths()

        val main = SequentialGroup(
            initCommands.farInit(),
            Delay(1.0),
            ShooterCommands.shootCommand(1.0),

            FollowPath(paths[0], true, 1.0),
            intakeAuto.autoIntakeNoGate(2.0),
            FollowPath(paths[1], true, 1.0),
            ShooterCommands.shootCommand(1.0),

            FollowPath(paths[2], true, 1.0),
            intakeAuto.autoIntakeNoGate(2.0),
            FollowPath(paths[3], true, 1.0),
            ShooterCommands.shootCommand(1.0),

            FollowPath(paths[4], true, 1.0),
            intakeAuto.autoIntakeNoGate(2.0),
            FollowPath(paths[5], true, 1.0),
            ShooterCommands.shootCommand(1.0),

            FollowPath(paths[6], true, 1.0),
            intakeAuto.autoIntakeNoGate(2.0),
            FollowPath(paths[7], true, 1.0),
            ShooterCommands.shootCommand(1.0)
        )
        main.schedule()
    }

    private fun buildPaths() {
        paths = arrayOf()

        val intakelow = follower.pathBuilder()
            .addPath(BezierLine(Pose(56.000, 9.000), Pose(10.000, 9.000)))
            .setLinearHeadingInterpolation(Math.toRadians(180.0), Math.toRadians(180.0))
            .build()

        val scorePose1 = follower.pathBuilder()
            .addPath(BezierLine(Pose(10.000, 9.000), Pose(56.000, 9.000)))
            .setLinearHeadingInterpolation(Math.toRadians(180.0), Math.toRadians(180.0))
            .build()

        val intakespike = follower.pathBuilder()
            .addPath(
                BezierCurve(
                    Pose(56.000, 9.000),
                    Pose(44.500, 40.000),
                    Pose(13.800, 36.300)
                )
            )
            .setLinearHeadingInterpolation(Math.toRadians(180.0), Math.toRadians(180.0))
            .build()

        val scorePose2 = follower.pathBuilder()
            .addPath(BezierLine(Pose(13.800, 36.300), Pose(56.000, 9.000)))
            .setLinearHeadingInterpolation(Math.toRadians(180.0), Math.toRadians(180.0))
            .build()

        val intakehigh = follower.pathBuilder()
            .addPath(BezierLine(Pose(56.000, 9.000), Pose(10.000, 18.500)))
            .setLinearHeadingInterpolation(Math.toRadians(180.0), Math.toRadians(180.0))
            .build()

        val scorePose3 = follower.pathBuilder()
            .addPath(BezierLine(Pose(10.000, 18.500), Pose(56.000, 9.000)))
            .setLinearHeadingInterpolation(Math.toRadians(180.0), Math.toRadians(180.0))
            .build()

        val intakemid = follower.pathBuilder()
            .addPath(BezierLine(Pose(56.000, 9.000), Pose(10.000, 13.750)))
            .setLinearHeadingInterpolation(Math.toRadians(180.0), Math.toRadians(180.0))
            .build()

        val scorePose4 = follower.pathBuilder()
            .addPath(BezierLine(Pose(10.000, 13.750), Pose(56.000, 9.000)))
            .setLinearHeadingInterpolation(Math.toRadians(180.0), Math.toRadians(180.0))
            .build()

        paths += intakelow
        paths += scorePose1
        paths += intakespike
        paths += scorePose2
        paths += intakehigh
        paths += scorePose3
        paths += intakemid
        paths += scorePose4
    }

    override fun onUpdate() {
        Drive.update()
        telemetry.update()
        telemetry.addData("pose",follower.pose)
    }

    override fun onStop() {
        initCommands.autoStop().schedule()
        Command.resetPose().schedule()

    }
}
