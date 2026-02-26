package org.firstinspires.ftc.teamcode.Lower.Gate

import dev.nextftc.core.commands.utility.InstantCommand
import dev.nextftc.core.subsystems.Subsystem
import dev.nextftc.hardware.impl.ServoEx
import org.firstinspires.ftc.teamcode.GateConstants

/**
 * Gate Subsystem
 * Simple open/close control for shooting gate
 *
 * Hardware: Standard servo on port "gate"
 */
object Gate : Subsystem {
    // ==================== HARDWARE ====================
    private var gateServo =  ServoEx("gate")

    // ==================== STATE ====================
    private var isOpen = false

    // ==================== INITIALIZATION ====================
    override fun initialize() {
    }

    // ==================== COMMANDS ====================
    /** Open the gate */
    val open = InstantCommand{
        gateServo.position = GateConstants.GATE_OPEN
        isOpen = true
    }


    /** Close the gate */
    val close = InstantCommand{
        gateServo.position = GateConstants.GATE_CLOSED

    }

    /** Toggle gate state */



    // ==================== PERIODIC ====================
    override fun periodic() {
        // No periodic updates needed for simple servo
    }
}