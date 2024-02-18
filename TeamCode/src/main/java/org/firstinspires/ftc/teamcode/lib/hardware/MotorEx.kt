package org.firstinspires.ftc.teamcode.lib.hardware

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior
import com.qualcomm.robotcore.hardware.DcMotorControllerEx
import com.qualcomm.robotcore.hardware.DcMotorImplEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.DcMotorSimple.Direction
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.PIDFCoefficients
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit
import org.firstinspires.ftc.teamcode.lib.units.Angle
import org.firstinspires.ftc.teamcode.lib.units.AngularVelocity
import org.firstinspires.ftc.teamcode.lib.units.rev
import org.firstinspires.ftc.teamcode.lib.units.revsec
import kotlin.math.abs
import kotlin.math.roundToInt
import kotlin.math.sign

@Suppress("MemberVisibilityCanBePrivate", "unused")
class MotorEx @JvmOverloads constructor(
    motor: DcMotorSimple,
    direction: Direction = motor.direction,
    cachingTolerance: Double = 0.0,
    private val ticksPerRev: Double = (motor as DcMotor).motorType.ticksPerRev
) {
    companion object {
        fun HardwareMap.motorEx(deviceName: String, direction: Direction? = null, cachingTolerance: Double = 0.0, ticksPerRev: Double? = null): MotorEx {
            val motor = get(DcMotor::class.java, deviceName)
            return MotorEx(
                motor = motor,
                direction = direction ?: motor.direction,
                cachingTolerance = cachingTolerance,
                ticksPerRev = ticksPerRev ?: motor.motorType.ticksPerRev
            )
        }

        @JvmOverloads
        fun gobilda312(motor: DcMotorSimple, direction: Direction = motor.direction, cachingTolerance: Double = 0.0) = MotorEx(
            motor = motor,
            direction = direction,
            cachingTolerance = cachingTolerance,
            ticksPerRev = (((1.0 + (46.0 / 17.0))) * (1.0 + (46.0 / 11.0))) * 28.0
        )

        fun HardwareMap.gobilda312(deviceName: String, direction: Direction? = null, cachingTolerance: Double = 0.0): MotorEx {
            val motor = get(DcMotor::class.java, deviceName)
            return gobilda312(
                motor = motor,
                direction = direction ?: motor.direction,
                cachingTolerance = cachingTolerance
            )
        }

        @JvmOverloads
        fun gobilda435(motor: DcMotorSimple, direction: Direction = motor.direction, cachingTolerance: Double = 0.0) = MotorEx(
            motor = motor,
            direction = direction,
            cachingTolerance = cachingTolerance,
            ticksPerRev = (((1.0 + (46.0 / 17.0))) * (1.0 + (46.0 / 17.0))) * 28.0
        )

        fun HardwareMap.gobilda435(deviceName: String, direction: Direction? = null, cachingTolerance: Double = 0.0): MotorEx {
            val motor = get(DcMotor::class.java, deviceName)
            return gobilda435(
                motor = motor,
                direction = direction ?: motor.direction,
                cachingTolerance = cachingTolerance
            )
        }

        @JvmOverloads
        fun rev12to1(motor: DcMotorSimple, direction: Direction = motor.direction, cachingTolerance: Double = 0.0) = MotorEx(
            motor = motor,
            direction = direction,
            cachingTolerance = cachingTolerance,
            ticksPerRev = (76.0 / 21.0) * (84.0 / 29.0) * 28.0
        )

        fun HardwareMap.rev12to1(deviceName: String, direction: Direction? = null, cachingTolerance: Double = 0.0): MotorEx {
            val motor = get(DcMotor::class.java, deviceName)
            return rev12to1(
                motor = motor,
                direction = direction ?: motor.direction,
                cachingTolerance = cachingTolerance
            )
        }
    }

    private val innerMotor: DcMotorImplEx = motor as DcMotorImplEx

    private var positionOffset: Int = 0

    private var cachedPower = 0.0
        set(value) {
            if (field != value) {
                innerMotor.power = value
            }

            field = value
        }

    val realPower by ::cachedPower

    val controller: DcMotorControllerEx = innerMotor.controller as DcMotorControllerEx

    val portNumber: Int = innerMotor.portNumber

    val motorType: MotorConfigurationType = innerMotor.motorType

    var cachingTolerance = cachingTolerance
        set(value) {
            val newValue = value.coerceIn(0.0, 2.0)

            if (field != newValue) {
                power = power
            }

            field = newValue
        }

    var power: Double = 0.0
        set(value) {
            val newPower = value.coerceIn(-1.0, 1.0)
            setPowerResult(newPower)
            field = newPower
        }

    var disabled: Boolean = innerMotor.isMotorEnabled.not()
        set(value) {
            if (field != value) {
                if (value) {
                    innerMotor.setMotorDisable()
                } else {
                    innerMotor.setMotorEnable()
                }
            }

            field = value
        }

    var direction: Direction = direction
        set(value) {
            if (field != value) {
                innerMotor.direction = value
            }

            field = value
        }

    var mode: DcMotor.RunMode = innerMotor.mode
        set(value) {
            if (field != value) {
                innerMotor.mode = value
            }

            field = value
        }

    var targetPositionTicksTolerance: Int = innerMotor.targetPositionTolerance
        set(value) {
            if (field != value) {
                innerMotor.targetPositionTolerance = value
            }
            field = value
        }

    var targetAngleTolerance: Angle
        get() = rev * (targetPositionTicksTolerance / ticksPerRev)
        set(value) {
            targetPositionTicksTolerance = (value.rev * ticksPerRev).roundToInt()
        }

    val positionTicks: Int get() = innerMotor.currentPosition - positionOffset

    val angle: Angle get() = rev * (positionTicks / ticksPerRev)

    var targetPositionTicks: Int = innerMotor.targetPosition
        set(value) {
            if (field != value) {
                innerMotor.targetPosition = value - positionOffset
            }
            field = value
        }

    var targetAngle: Angle
        get() = rev * (targetPositionTicks / ticksPerRev)
        set(value) {
            targetPositionTicks = (value.rev * ticksPerRev).roundToInt()
        }

    val velocityTicks: Double get() = innerMotor.velocity

    val angularVelocity: AngularVelocity get() = revsec * (velocityTicks / ticksPerRev)

    var targetVelocityTicks: Double = 0.0
        set(value) {
            if (value != field) {
                innerMotor.velocity = value
            }

            field = value
        }

    var targetAnglularVelocity: AngularVelocity
        get() = revsec * (targetVelocityTicks / ticksPerRev)
        set(value) {
            targetVelocityTicks = value.revsec * ticksPerRev
        }

    var zeroPowerBehavior: ZeroPowerBehavior = innerMotor.zeroPowerBehavior
        set(value) {
            if (field != value) {
                innerMotor.zeroPowerBehavior = value
            }

            field = value
        }

    val powerFloat: Boolean get() = zeroPowerBehavior == ZeroPowerBehavior.FLOAT && power == 0.0

    val isBusy: Boolean get() = innerMotor.isBusy

    val isOverCurrent get() = innerMotor.isOverCurrent

    init {
        mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        zeroPowerBehavior = ZeroPowerBehavior.BRAKE
        innerMotor.direction = direction
        innerMotor.power = 0.0
    }

    fun resetPosition() {
        positionOffset = innerMotor.currentPosition
    }

    fun setPowerResult(power: Double): Boolean {
        val newPower = power.coerceIn(-1.0, 1.0)
        val overChangeThreshold = abs(newPower - cachedPower) >= cachingTolerance
        val targetingFullPower = (newPower >= 1.0 && cachedPower < 1.0) || (newPower <= -1.0 && cachedPower > -1.0)
        val changedDirectionOrBrake = newPower.sign != cachedPower.sign
        return if (overChangeThreshold || targetingFullPower || changedDirectionOrBrake) {
            cachedPower = newPower
            true
        } else {
            false
        }
    }

    fun getVelocity(unit: AngleUnit) = innerMotor.getVelocity(unit)

    fun getCurrent(unit: CurrentUnit) = innerMotor.getCurrent(unit)

    fun getCurrentAlert(unit: CurrentUnit) = innerMotor.getCurrentAlert(unit)

    fun setCurrentAlert(current: Double, unit: CurrentUnit) = innerMotor.setCurrentAlert(current, unit)

    fun setPIDFCoefficients(mode: DcMotor.RunMode, pidfCoefficients: PIDFCoefficients) = innerMotor.setPIDFCoefficients(mode, pidfCoefficients)

    fun getPIDFCoefficients(mode: DcMotor.RunMode): PIDFCoefficients = innerMotor.getPIDFCoefficients(mode)

    fun setVelocityPIDFCoefficients(p: Double, i: Double, d: Double, f: Double) = innerMotor.setVelocityPIDFCoefficients(p, i, d, f)

    fun setPositionPIDFCoefficients(p: Double) = innerMotor.setPositionPIDFCoefficients(p)
}