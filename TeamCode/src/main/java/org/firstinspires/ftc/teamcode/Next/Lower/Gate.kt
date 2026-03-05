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

    // ==================== INITIALIZATION ====================
    override fun initialize() {

    }

    // ==================== COMMANDS ====================
    /** Open the gate */
    internal fun setPosition(pos: Double){
        gateServo.position = pos
    }

    val open = InstantCommand{
        setPosition(GateConstants.GATE_OPEN)
    }
    val close = InstantCommand{
        setPosition(GateConstants.GATE_CLOSED)
    }
    /** Toggle gate state */

    fun open(){
        setPosition(GateConstants.GATE_OPEN)
    }
    fun close(){
        setPosition(GateConstants.GATE_CLOSED)
    }

    // ==================== PERIODIC ====================
    override fun periodic() {
        // No periodic updates needed for simple servo
    }
}