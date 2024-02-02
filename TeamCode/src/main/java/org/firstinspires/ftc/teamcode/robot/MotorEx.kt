package org.firstinspires.ftc.teamcode.robot

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotor.RunMode
import com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior
import com.qualcomm.robotcore.hardware.DcMotorControllerEx
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
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

@Suppress("unused", "MemberVisibilityCanBePrivate")
class MotorEx(
    dcMotor: DcMotor,
    private val changeThreshold: Double = 0.02,
    private val ticksPerRev: Double = dcMotor.motorType.ticksPerRev
) {
    private val motor: DcMotorEx = dcMotor as DcMotorEx

    private var cachedPower: Double = motor.power

    val controller: DcMotorControllerEx = motor.controller as DcMotorControllerEx

    val portNumber: Int = motor.portNumber

    var power: Double = motor.power
        set(value) {
            val newPower = value.coerceIn(-1.0, 1.0)
            val overChangeThreshhold = abs(newPower - cachedPower) >= changeThreshold
            val targetingFullPower = (newPower >= 1.0 && cachedPower < 1.0) || (newPower <= -1.0 && cachedPower > -1.0)
            val changedDirectionOrBrake = newPower.sign != cachedPower.sign
            if (overChangeThreshhold || targetingFullPower || changedDirectionOrBrake) {
                motor.power = newPower
                cachedPower = newPower
            }
            field = newPower
        }

    fun setPowerResult(value: Double): Boolean {
        val newPower = value.coerceIn(-1.0, 1.0)
        val overChangeThreshhold = abs(newPower - cachedPower) >= changeThreshold
        val targetingFullPower = (newPower >= 1.0 && cachedPower < 1.0) || (newPower <= -1.0 && cachedPower > -1.0)
        val changedDirectionOrBrake = newPower.sign != cachedPower.sign
        return if (overChangeThreshhold || targetingFullPower || changedDirectionOrBrake) {
            motor.power = newPower
            cachedPower = newPower
            power = newPower
            true
        } else {
            power = newPower
            false
        }
    }

    var disabled: Boolean = !motor.isMotorEnabled
        set(value) {
            if (value != field) {
                if (value) {
                    motor.setMotorDisable()
                } else {
                    motor.setMotorEnable()
                }
            }
            field = value
        }

    var direction: DcMotorSimple.Direction = motor.direction
        set(value) {
            if (value != field) {
                motor.direction = value
            }
            field = value
        }

    var type: MotorConfigurationType = motor.motorType
        set(value) {
            if (value != field) {
                motor.motorType = value
            }
            field = value
        }

    val isOverCurrent: Boolean
        get() = motor.isOverCurrent

    var targetPositionTolerance: Int = motor.targetPositionTolerance
        set(value) {
            if (value != field) {
                motor.targetPositionTolerance = value
            }
            field = value
        }

    val positionTicks: Int get() = motor.currentPosition

    val angle: Angle get() = rev * (positionTicks / ticksPerRev)

    var targetPositionTicks: Int = motor.targetPosition
        set(value) {
            if (value != field) {
                motor.targetPosition = value
            }
            field = value
        }

    var targetAngle: Angle
        get() = rev * (targetPositionTicks / ticksPerRev)
        set(value) {
            targetPositionTicks = (value.rev * ticksPerRev).roundToInt()
        }

    val velocityTicks: Double get() = motor.velocity

    val angularVelocity: AngularVelocity get() = revsec * (velocityTicks / ticksPerRev)

    fun getVelocity(unit: AngleUnit) = motor.getVelocity(unit)

    var targetVelocityTicks: Double = 0.0
        set(value) {
            if (value != field) {
                motor.velocity = value
            }
            field = value
        }

    var targetAngularVelocity: AngularVelocity
        get() = revsec * (targetVelocityTicks / ticksPerRev)
        set(value) {
            targetVelocityTicks = value.revsec * ticksPerRev
        }

    var zeroPowerBehavior: ZeroPowerBehavior = motor.zeroPowerBehavior
        set(value) {
            if (value != field) {
                motor.zeroPowerBehavior = value
            }
            field = value
        }

    val powerFloat: Boolean get() = zeroPowerBehavior == ZeroPowerBehavior.FLOAT && power == 0.0

    val isBusy: Boolean get() = motor.isBusy

    var mode: RunMode = motor.mode
        set(value) {
            if (field != value) {
                motor.mode = value
            }
            field = value
        }

    fun getCurrent(unit: CurrentUnit) = motor.getCurrent(unit)

    fun getCurrentAlert(unit: CurrentUnit) = motor.getCurrentAlert(unit)

    fun setCurrentAlert(current: Double, unit: CurrentUnit) = motor.setCurrentAlert(current, unit)

    fun setPIDFCoefficients(mode: RunMode, pidfCoefficients: PIDFCoefficients) = motor.setPIDFCoefficients(mode, pidfCoefficients)

    fun getPIDFCoefficients(mode: RunMode): PIDFCoefficients = motor.getPIDFCoefficients(mode)

    fun setVelocityPIDFCoefficients(p: Double, i: Double, d: Double, f: Double) = motor.setVelocityPIDFCoefficients(p, i, d, f)

    fun setPositionPIDFCoefficients(p: Double) = motor.setPositionPIDFCoefficients(p)

    init {
        motor.direction
    }
}