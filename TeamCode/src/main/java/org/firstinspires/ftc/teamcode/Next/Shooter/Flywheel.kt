package org.firstinspires.ftc.teamcode.subsystem

import com.bylazar.configurables.annotations.Configurable
import com.bylazar.telemetry.JoinedTelemetry
import com.bylazar.telemetry.PanelsTelemetry
import com.qualcomm.robotcore.hardware.VoltageSensor
import dev.nextftc.control.ControlSystem
import dev.nextftc.control.KineticState
import dev.nextftc.control.builder.controlSystem
import dev.nextftc.control.feedback.PIDCoefficients
import dev.nextftc.control.feedforward.BasicFeedforwardParameters
import dev.nextftc.core.commands.Command
import dev.nextftc.core.commands.utility.InstantCommand
import dev.nextftc.core.subsystems.Subsystem
import dev.nextftc.hardware.impl.MotorEx
import dev.nextftc.ftc.ActiveOpMode
import dev.nextftc.ftc.ActiveOpMode.telemetry
import java.util.function.Supplier

@Configurable
object FlyWheel : Subsystem {

    // Hardware
    lateinit var motor1: MotorEx
    private lateinit var motor2: MotorEx

    // FIX: lazy so hardware map is available at access time
    private val battery: VoltageSensor by lazy {
        ActiveOpMode.hardwareMap.get(VoltageSensor::class.java, "Control Hub")
    }

    // ==================== TUNABLE COEFFICIENTS ====================
    // Changes to these take effect on the next periodic() call because
    // the controller is rebuilt from them each loop.

    @JvmField  var ffCoefficients  = BasicFeedforwardParameters(0.001, 0.006, 0.0)
    @JvmField  var pidCoefficients = PIDCoefficients(0.015, 0.00, 0.01)



    // FIX: controller is now rebuilt in periodic() using the current coefficients,
    // so @JvmField / dashboard edits to ffCoefficients / pidCoefficients take effect
    // immediately without restarting the op mode.
    private var controller: ControlSystem = buildController()

    private fun buildController(): ControlSystem = controlSystem {
        basicFF(ffCoefficients)
        velPid(pidCoefficients)
    }

    // ==================== VOLTAGE COMPENSATION ====================
    private const val V_NOMINAL = 12.0
    var voltFilt = 12.0
    private const val ALPHA_VOLT = 0.08

    @JvmField var voltageCompEnabled = true

    // ==================== STATE ====================
    var targetVelocity = 0.0

    // ==================== INITIALIZATION ====================
    override fun initialize() {
        motor1 = MotorEx("Fly1").floatMode()
        motor2 = MotorEx("Fly2").floatMode()
        voltFilt = 12.0
        targetVelocity = 0.0
        controller = buildController()
    }

    // ==================== VELOCITY CONTROL ====================

    /**
     * Set target velocity with voltage compensation factored into the goal.
     * The controller goal is set in raw velocity units; periodic() applies
     * voltage comp to the output power so the motor receives the right drive
     * regardless of battery state.
     */
    fun setVelocity(speed: Double) {
        targetVelocity = speed
        // FIX: rebuild controller so any dashboard-tuned coefficients are picked up
        controller = buildController()
        controller.goal = KineticState(0.0, speed)
    }

    // ==================== PRESETS ====================
    // FIX: changed from val to fun so commands are built fresh after initialize(),
    // not at class-load time before the hardware map exists.
    val off      = InstantCommand { setVelocity(0.0) }
    val close  = InstantCommand { setVelocity(1000.0) }
    val mid      = InstantCommand { setVelocity(1250.0) }
    val far     = InstantCommand { setVelocity(1500.0) }
    val max      = InstantCommand { setVelocity(1500.0) }
    val maxFar  = InstantCommand { setVelocity(1600.0) }
    val idle     = InstantCommand { setVelocity(-300.0) }
    val runHigh  = InstantCommand { setVelocity(1900.0) }

    // ==================== MOTOR CONTROL ====================

    private fun setMotorPowers(power: Double) {
        val clamped = power.coerceIn(-0.85, 0.85)
        motor1.power = clamped
        motor2.power = clamped
    }

    // ==================== PERIODIC ====================

    override fun periodic() {
        // Filter battery voltage
        val voltRaw = battery.voltage.coerceAtLeast(9.0)
        voltFilt += ALPHA_VOLT * (voltRaw - voltFilt)
        val voltageRatio = V_NOMINAL / voltFilt

        // Raw PID+FF output from controller (uses motor1 as the velocity source)
        val rawPower = controller.calculate(motor1.state)

        // FIX: voltage comp applied consistently here for ALL velocity modes —
        // whether set by setVelocity(), a preset, or an external caller.
        val finalPower = if (voltageCompEnabled) {
            (rawPower * voltageRatio).coerceIn(-0.85, 0.85)
        } else {
            rawPower.coerceIn(-0.85, 0.85)
        }


        setMotorPowers(finalPower)

        // ── Panels telemetry ──────────────────────────────────────────
        telemetry.addData("Flywheel/Power",        "%.3f".format(finalPower))
        telemetry.addData("Flywheel/Target Vel",   "%.1f".format(targetVelocity))
        telemetry.addData("Flywheel/Actual Vel",   "%.1f".format(motor1.velocity))
        telemetry.addData("Flywheel/Vel Error",    "%.1f".format(targetVelocity - motor1.velocity))
        telemetry.addData("Flywheel/At Target",    isAtTarget())
        telemetry.addData("Flywheel/Voltage",      "%.2f".format(voltFilt))
        telemetry.addData("Flywheel/Volt Ratio",   "%.3f".format(voltageRatio))
        telemetry.addData("Flywheel/Volt Comp On", voltageCompEnabled)
    }

    // ==================== COMMANDS ====================

    /**
     * Manual power override — bypasses PID/FF entirely.
     * Voltage comp is still applied so manual feel is consistent across battery levels.
     */
    class Manual(private val shooterPower: Supplier<Double>) : Command() {
        override val isDone = false
        init { requires(FlyWheel) }
        override fun update() {
            val voltageRatio = if (FlyWheel.voltageCompEnabled) V_NOMINAL / FlyWheel.voltFilt else 1.0
            val compensated = (shooterPower.get() * voltageRatio).coerceIn(-0.85, 0.85)
            FlyWheel.setMotorPowers(compensated)
        }
    }

    // ==================== STATUS ====================

    fun isAtTarget(): Boolean =
        motor1.velocity > (targetVelocity - 20.0) &&
                motor1.velocity < (targetVelocity + 40.0)

    fun getVelocity(): Double = motor1.velocity


}