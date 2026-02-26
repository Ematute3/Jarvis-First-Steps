package org.firstinspires.ftc.teamcode.TeleOp

import com.bylazar.telemetry.JoinedTelemetry
import com.bylazar.telemetry.PanelsTelemetry
import com.pedropathing.geometry.Pose
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import dev.nextftc.core.commands.groups.ParallelGroup
import dev.nextftc.core.components.BindingsComponent
import dev.nextftc.core.components.SubsystemComponent
import dev.nextftc.extensions.pedro.PedroComponent
import dev.nextftc.extensions.pedro.PedroDriverControlled
import dev.nextftc.ftc.Gamepads
import dev.nextftc.ftc.NextFTCOpMode
import dev.nextftc.ftc.components.BulkReadComponent
import dev.nextftc.ftc.ActiveOpMode.telemetry
import org.firstinspires.ftc.teamcode.Lower.Drive.Drive
import org.firstinspires.ftc.teamcode.Lower.Gate.Gate
import org.firstinspires.ftc.teamcode.Lower.Intake.Intake
import org.firstinspires.ftc.teamcode.Shooter.FlyWheel.FlyWheel
import org.firstinspires.ftc.teamcode.Shooter.Hood.Hood
import org.firstinspires.ftc.teamcode.Shooter.Limelight.Limelight
import org.firstinspires.ftc.teamcode.AutoAim.AutoAim
import org.firstinspires.ftc.teamcode.Next.Shooter.Turret
import org.firstinspires.ftc.teamcode.pedroPathing.Constants

/**
 * Blue Alliance TeleOp
 *
 * CONTROLS:
 * - Driver (Gamepad 1):
 *   - Left Stick: Field-centric drive
 *   - Right Stick X: Rotation
 *   - Left Trigger: Intake
 *   - Right Bumper: Fire (hold)
 *   - Right Trigger: Manual fire (when at target)
 *   - D-Pad: Flywheel presets (up=far, right=mid, down=close, left=off)
 *   - X: Toggle auto-aim
 *   - Start: Toggle alliance
 *   - Back: Reset all
 *
 * - Operator (Gamepad 2):
 *   - Left Stick X: Manual turret
 *   - Left Bumper: Toggle aim mode
 *   - Right Bumper: Reset turret
 *   - D-Pad: Hood presets
 */
@TeleOp(name = "TeleOp - Blue", group = "Competition")
class TeleOpBlue : NextFTCOpMode() {

    private val panelsTelemetry = PanelsTelemetry.ftcTelemetry
    private val joinedTelemetry = JoinedTelemetry(telemetry, panelsTelemetry)

    // ==================== ENUMS ====================
    private enum class AimMode { OFF, ODO }
    private enum class FlyMode { IDLE, CLOSE, MID, FAR, AUTO }
    private enum class ShootState { READY, SPINNING, FIRING }

    // ==================== STATE ====================
    private var currentAimMode = AimMode.ODO
    private var currentFlyMode = FlyMode.AUTO
    private var shootState = ShootState.READY
    private var autoAimEnabled = false

    // ==================== INITIALIZATION ====================
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
        // Set to blue alliance
        Drive.alliance = Drive.Alliance.BLUE
        Drive.lastKnown = Pose(0.0, 0.0, 0.0)
    }

    override fun onStartButtonPressed() {
        // Start drivetrain - field centric
        PedroDriverControlled(
            -Gamepads.gamepad1.leftStickY,
            -Gamepads.gamepad1.leftStickX,
            -Gamepads.gamepad1.rightStickX,
            false
        ).schedule()

        // Reset turret at start


        // Default to auto aim
        autoAimEnabled = true
        currentAimMode = AimMode.ODO

        bindControls()
    }

    // ==================== CONTROLS ====================
    private fun bindControls() {
        // ==================== DRIVER (GAMEPAD 1) ====================

        // --- INTAKE ---
        // Left Trigger: Intake
        Gamepads.gamepad1.leftTrigger.greaterThan(0.5)
            .whenBecomesTrue(Intake.run)
            .whenBecomesFalse(Intake.stop)

        // Left Bumper: Reverse/Eject
        Gamepads.gamepad1.leftBumper
            .whenBecomesTrue(Intake.reverse)
            .whenBecomesFalse(Intake.stop)

        // --- SHOOTING ---
        // Right Bumper: Fire (hold)
        Gamepads.gamepad1.rightBumper
            .whenBecomesTrue { fire() }
            .whenBecomesFalse {
                Gate.close
                shootState = ShootState.READY
            }

        // Right Trigger: Manual fire (when at target)
        Gamepads.gamepad1.rightTrigger.greaterThan(0.5)
            .whenBecomesTrue {
                if (FlyWheel.isAtTarget()) {
                    fire()
                }
            }

        // --- FLYWHEEL PRESETS ---
        // D-Pad Up: Far
        Gamepads.gamepad1.dpadUp.whenBecomesTrue {
            FlyWheel.far()
            currentFlyMode = FlyMode.FAR
        }

        // D-Pad Right: Mid
        Gamepads.gamepad1.dpadRight.whenBecomesTrue {
            FlyWheel.mid()
            currentFlyMode = FlyMode.MID
        }

        // D-Pad Down: Close
        Gamepads.gamepad1.dpadDown.whenBecomesTrue {
            FlyWheel.close()
            currentFlyMode = FlyMode.CLOSE
        }

        // D-Pad Left: Off
        Gamepads.gamepad1.dpadLeft.whenBecomesTrue {
            FlyWheel.off()
            currentFlyMode = FlyMode.IDLE
        }

        // --- AUTO-AIM ---
        // Cross (X): Toggle auto-aim
        Gamepads.gamepad1.cross.whenBecomesTrue {
            autoAimEnabled = !autoAimEnabled
            AutoAim.setAutoAim(autoAimEnabled)
        }

        // --- ALLIANCE ---
        // Start: Toggle alliance (for testing)
        Gamepads.gamepad1.start.whenBecomesTrue {
            // Already blue, could add toggle logic
        }

        // --- RESET ---
        // Back: Reset all
        Gamepads.gamepad1.back.whenBecomesTrue { resetAll() }

        // ==================== OPERATOR (GAMEPAD 2) ====================

        // --- TURRET ---
        // Left Stick X: Manual turret


        // Left Bumper: Toggle aim mode
        Gamepads.gamepad2.leftBumper.whenBecomesTrue {
            currentAimMode = when (currentAimMode) {
                AimMode.OFF -> AimMode.ODO
                AimMode.ODO -> AimMode.OFF
            }
        }

        // Right Bumper: Reset turret


        // --- HOOD ---
        // D-Pad: Hood presets
        Gamepads.gamepad2.dpadUp.whenBecomesTrue { Hood.far() }
        Gamepads.gamepad2.dpadDown.whenBecomesTrue { Hood.close() }
        Gamepads.gamepad2.dpadLeft.whenBecomesTrue { Hood.mid() }
        Gamepads.gamepad2.dpadRight.whenBecomesTrue { Hood.autoAdjust() }
    }

    // ==================== FIRE ====================
    private fun fire() {
        if (FlyWheel.isAtTarget()) {
            Gate.open
            shootState = ShootState.FIRING
        } else {
            shootState = ShootState.SPINNING
        }
    }

    // ==================== RESET ====================
    private fun resetAll() {
        ParallelGroup(
            Hood.close,
            Gate.close,
            FlyWheel.off,
            Intake.stop,

        ).schedule()

        currentFlyMode = FlyMode.IDLE
        autoAimEnabled = false
        AutoAim.setAutoAim(false)
    }

    // ==================== UPDATE LOOP ====================
    override fun onUpdate() {
        // Update drive position
        Drive.update()

        // Update limelight
        Limelight.update()

        // Update auto aim if enabled
        if (autoAimEnabled) {
            AutoAim.update()
        }

        // Update turret based on aim mode
        when (currentAimMode) {
            AimMode.OFF -> { /* Manual control */ }
            AimMode.ODO -> Turret.lockOn()
        }

        // Update telemetry
        updateTelemetry()
    }

    // ==================== TELEMETRY ====================
    private fun updateTelemetry() {
        // Position
        telemetry.addData("=== BLUE TELEOP ===", "")
        telemetry.addData("Pose/X", "%.1f".format(Drive.currentX))
        telemetry.addData("Pose/Y", "%.1f".format(Drive.currentY))
        telemetry.addData("Pose/Heading", "%.1f°".format(Math.toDegrees(Drive.currentHeading)))

        // Shooter
        telemetry.addData("Flywheel/Target", "%.0f".format(FlyWheel.targetVelocity))
        telemetry.addData("Flywheel/Actual", "%.0f".format(FlyWheel.getVelocity()))
        telemetry.addData("Flywheel/At Target", if (FlyWheel.isAtTarget()) "YES" else "NO")

        // Turret

        telemetry.addData("Turret/Mode", currentAimMode.name)

        // Hood
        telemetry.addData("Hood/Position", "%.2f".format(Hood.currentPosition))
        telemetry.addData("Hood/Preset", Hood.currentPreset.name)

        // Auto Aim
        telemetry.addData("AutoAim", if (autoAimEnabled) "ON" else "OFF")
        telemetry.addData("Zone", if (Drive.isInShootingZone()) "YES" else "NO")

        // Limelight
        Limelight.updateTelemetry()

        panelsTelemetry.update()
    }
}