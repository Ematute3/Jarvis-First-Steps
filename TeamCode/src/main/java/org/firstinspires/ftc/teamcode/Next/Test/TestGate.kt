package org.firstinspires.ftc.teamcode.Test

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import dev.nextftc.core.components.BindingsComponent
import dev.nextftc.core.components.SubsystemComponent
import dev.nextftc.ftc.Gamepads
import dev.nextftc.ftc.NextFTCOpMode
import dev.nextftc.ftc.components.BulkReadComponent

import org.firstinspires.ftc.teamcode.Lower.Gate.Gate

/**
 * Test Gate Opmode
 *
 * Tests gate servo open/close
 *
 * CONTROLS:
 * - A Button: Toggle gate
 * - X Button: Open gate
 * - B Button: Close gate
 */
@TeleOp(name = "TEST - Gate", group = "Test")
class TestGate : NextFTCOpMode() {

    init {
        addComponents(
            SubsystemComponent(Gate),
            BulkReadComponent,
            BindingsComponent
        )
    }

    override fun onStartButtonPressed() {
        bindControls()
    }

    private fun bindControls() {
        // A: Toggle

        // X: Open
        Gamepads.gamepad1.x whenBecomesTrue Gate.open whenBecomesFalse Gate.close


        // B: Close

    }

    override fun onUpdate() {
        telemetry.addData("=== GATE TEST ===", "")
        telemetry.addData("", "")
        telemetry.addData("Controls:", "")
        telemetry.addData("A", "Toggle")
        telemetry.addData("X", "Open")
        telemetry.addData("B", "Close")
        telemetry.update()
    }
}