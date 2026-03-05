@file:Suppress("PackageName")

package org.firstinspires.ftc.teamcode.Systems

import com.pedropathing.geometry.Pose
import dev.nextftc.core.commands.Command
import dev.nextftc.core.commands.delays.Delay
import dev.nextftc.core.commands.groups.SequentialGroup
import dev.nextftc.core.commands.utility.InstantCommand
import dev.nextftc.core.subsystems.SubsystemGroup
import dev.nextftc.extensions.pedro.PedroComponent.Companion.follower
import org.firstinspires.ftc.teamcode.HoodConstants
import org.firstinspires.ftc.teamcode.Lower.Drive.Drive

import org.firstinspires.ftc.teamcode.Next.Shooter.FlyWheel
import org.firstinspires.ftc.teamcode.Lower.Gate.Gate
import org.firstinspires.ftc.teamcode.Lower.Intake.Intake
import org.firstinspires.ftc.teamcode.Next.Shooter.Turret
import org.firstinspires.ftc.teamcode.Shooter.Hood.Hood
import org.firstinspires.inspection.InspectionState
import java.time.Instant

object ShooterCommands : SubsystemGroup(FlyWheel, Hood, Gate) {

    fun shootCommand(waitTime: Double): Command =
        SequentialGroup(
            Delay(0.5),
            InstantCommand { Gate.open() },
            InstantCommand{Intake.run(1.0)},
            Delay(waitTime),
            InstantCommand { Intake.run(0.0) },
            InstantCommand{Gate.close()}

        )


}
object initCommands : SubsystemGroup(FlyWheel, Hood, Gate, Turret){
    fun midInit(): Command =
        SequentialGroup(

                InstantCommand{FlyWheel.setVelocity(1500.0)},
                    InstantCommand{Gate.close()},
                        InstantCommand{Hood.setPosition(HoodConstants.HOOD_MID)}

        )
    fun farInit(): Command =
        SequentialGroup(

                InstantCommand{FlyWheel.setVelocity(1900.0)},
                        InstantCommand{Gate.close()},
                    InstantCommand{Hood.setPosition(HoodConstants.HOOD_FAR)},
                        InstantCommand{Turret.lock()}

        )

    fun autoStop(): Command =
        SequentialGroup(

            InstantCommand{FlyWheel.setVelocity(0.0)  },
                InstantCommand{ Gate.close()  },
                    InstantCommand{ Intake.run(0.0)  },
                        InstantCommand{Hood.setPosition(HoodConstants.HOOD_CLOSE)}

        )
}
object intakeAuto: SubsystemGroup(Intake, Gate) {
    fun intakeStop(): Command =
        SequentialGroup(
            InstantCommand { Intake.run(0.0) },
            InstantCommand {
                Gate.close()
            }
        )

    fun autoIntake(wait: Double): Command =
        SequentialGroup(
            InstantCommand {
                Intake.run(1.0)
            },
            InstantCommand {
                Gate.open()
            },
            Delay(wait),
            intakeStop()


        )

    fun autoIntakeNoGate(wait: Double): Command =
        SequentialGroup(
            InstantCommand {
                Intake.run(1.0)
            },
            Delay(wait),
            intakeStop()

        )
}
object Command: SubsystemGroup(Drive, Turret) {
    var autoPose = Pose(0.0, 0.0, 0.0)
    fun resetPose(): Command =
        InstantCommand {
    autoPose = follower.pose
    }

}



