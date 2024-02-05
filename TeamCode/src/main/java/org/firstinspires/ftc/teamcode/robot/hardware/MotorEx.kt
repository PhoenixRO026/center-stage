package org.firstinspires.ftc.teamcode.robot.hardware

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotor.RunMode
import com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior
import com.qualcomm.robotcore.hardware.DcMotorControllerEx
import com.qualcomm.robotcore.hardware.DcMotorEx
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

@Suppress("unused", "MemberVisibilityCanBePrivate")
class MotorEx @JvmOverloads constructor(
    dcMotor: DcMotor,
    motorDirection: Direction = dcMotor.direction,
    changeThreshold: Double = 0.02,
    private val ticksPerRev: Double = dcMotor.motorType.ticksPerRev,
    private val onPowerUpdate: () -> Unit = {}
) {
    companion object {
        @JvmOverloads
        fun gobilda312(dcMotor: DcMotor, direction: Direction = dcMotor.direction, onPowerUpdate: () -> Unit = {}, changeThreshold: Double = 0.02) = MotorEx(
            dcMotor = dcMotor,
            motorDirection = direction,
            changeThreshold = changeThreshold,
            ticksPerRev = (((1.0 + (46.0 / 17.0))) * (1.0 + (46.0 / 11.0))) * 28.0,
            onPowerUpdate = onPowerUpdate
        )

        fun HardwareMap.gobilda312(deviceName: String, direction: Direction? = null, onPowerUpdate: () -> Unit = {}, changeThreshold: Double = 0.02): MotorEx {
            val motor = get(DcMotor::class.java, deviceName)
            return gobilda312(
                dcMotor = motor,
                direction = direction ?: motor.direction,
                changeThreshold = changeThreshold,
                onPowerUpdate = onPowerUpdate
            )
        }

        @JvmOverloads
        fun gobilda435(dcMotor: DcMotor, direction: Direction = dcMotor.direction, onPowerUpdate: () -> Unit = {}, changeThreshold: Double = 0.02) = MotorEx(
            dcMotor = dcMotor,
            motorDirection = direction,
            changeThreshold = changeThreshold,
            ticksPerRev = (((1.0 + (46.0 / 17.0))) * (1.0 + (46.0 / 17.0))) * 28.0,
            onPowerUpdate = onPowerUpdate
        )

        fun HardwareMap.gobilda435(deviceName: String, direction: Direction? = null, onPowerUpdate: () -> Unit = {}, changeThreshold: Double = 0.02): MotorEx {
            val motor = get(DcMotor::class.java, deviceName)
            return gobilda435(
                dcMotor = motor,
                direction = direction ?: motor.direction,
                changeThreshold = changeThreshold,
                onPowerUpdate = onPowerUpdate
            )
        }

        @JvmOverloads
        fun rev12to1(dcMotor: DcMotor, direction: Direction = dcMotor.direction, onPowerUpdate: () -> Unit = {}, changeThreshold: Double = 0.02) = MotorEx(
            dcMotor = dcMotor,
            motorDirection = direction,
            changeThreshold = changeThreshold,
            ticksPerRev = (76.0 / 21.0) * (84.0 / 29.0) * 28.0,
            onPowerUpdate = onPowerUpdate
        )

        fun HardwareMap.rev12to1(deviceName: String, direction: Direction? = null, onPowerUpdate: () -> Unit = {}, changeThreshold: Double = 0.02): MotorEx {
            val motor = get(DcMotor::class.java, deviceName)
            return rev12to1(
                dcMotor = motor,
                direction = direction ?: motor.direction,
                changeThreshold = changeThreshold,
                onPowerUpdate = onPowerUpdate
            )
        }
    }

    private val changeThreshold: Double = changeThreshold.coerceAtLeast(0.0)

    private var cachedPower: Double = 0.0
        set(value) {
            if (field != value) {
                internalMotor.power = value
                onPowerUpdate()
            }
            field = value
        }

    val internalMotor: DcMotorEx = dcMotor as DcMotorEx

    val controller: DcMotorControllerEx = internalMotor.controller as DcMotorControllerEx

    val portNumber: Int = internalMotor.portNumber

    var power: Double = 0.0
        set(value) {
            val newPower = value.coerceIn(-1.0, 1.0)
            val overChangeThreshold = abs(newPower - cachedPower) >= changeThreshold
            val targetingFullPower = (newPower >= 1.0 && cachedPower < 1.0) || (newPower <= -1.0 && cachedPower > -1.0)
            val changedDirectionOrBrake = newPower.sign != cachedPower.sign
            if (overChangeThreshold || targetingFullPower || changedDirectionOrBrake) {
                cachedPower = newPower
            }
            field = newPower
        }

    val realPower: Double by ::cachedPower

    fun setPowerResult(value: Double): Boolean {
        val newPower = value.coerceIn(-1.0, 1.0)
        val overChangeThreshold = abs(newPower - cachedPower) >= changeThreshold
        val targetingFullPower = (newPower >= 1.0 && cachedPower < 1.0) || (newPower <= -1.0 && cachedPower > -1.0)
        val changedDirectionOrBrake = newPower.sign != cachedPower.sign
        return if (overChangeThreshold || targetingFullPower || changedDirectionOrBrake) {
            cachedPower = newPower
            power = newPower
            true
        } else {
            power = newPower
            false
        }
    }

    var disabled: Boolean = !internalMotor.isMotorEnabled
        set(value) {
            if (value != field) {
                if (value) {
                    internalMotor.setMotorDisable()
                } else {
                    internalMotor.setMotorEnable()
                }
            }
            field = value
        }

    var direction: Direction = internalMotor.direction
        set(value) {
            if (value != field) {
                internalMotor.direction = value
            }
            field = value
        }

    var type: MotorConfigurationType = internalMotor.motorType
        set(value) {
            if (value != field) {
                internalMotor.motorType = value
            }
            field = value
        }

    val isOverCurrent: Boolean get() = internalMotor.isOverCurrent

    var targetPositionTolerance: Int = internalMotor.targetPositionTolerance
        set(value) {
            if (value != field) {
                internalMotor.targetPositionTolerance = value
            }
            field = value
        }

    val positionTicks: Int get() = internalMotor.currentPosition

    val angle: Angle get() = rev * (positionTicks / ticksPerRev)

    var targetPositionTicks: Int = internalMotor.targetPosition
        set(value) {
            if (value != field) {
                internalMotor.targetPosition = value
            }
            field = value
        }

    var targetAngle: Angle
        get() = rev * (targetPositionTicks / ticksPerRev)
        set(value) {
            targetPositionTicks = (value.rev * ticksPerRev).roundToInt()
        }

    val velocityTicks: Double get() = internalMotor.velocity

    val angularVelocity: AngularVelocity get() = revsec * (velocityTicks / ticksPerRev)

    fun getVelocity(unit: AngleUnit) = internalMotor.getVelocity(unit)

    var targetVelocityTicks: Double = 0.0
        set(value) {
            if (value != field) {
                internalMotor.velocity = value
            }
            field = value
        }

    var targetAngularVelocity: AngularVelocity
        get() = revsec * (targetVelocityTicks / ticksPerRev)
        set(value) {
            targetVelocityTicks = value.revsec * ticksPerRev
        }

    var zeroPowerBehavior: ZeroPowerBehavior = internalMotor.zeroPowerBehavior
        set(value) {
            if (value != field) {
                internalMotor.zeroPowerBehavior = value
            }
            field = value
        }

    val powerFloat: Boolean get() = zeroPowerBehavior == ZeroPowerBehavior.FLOAT && power == 0.0

    val isBusy: Boolean get() = internalMotor.isBusy

    var mode: RunMode = internalMotor.mode
        set(value) {
            if (field != value) {
                internalMotor.mode = value
            }
            field = value
        }

    fun getCurrent(unit: CurrentUnit) = internalMotor.getCurrent(unit)

    fun getCurrentAlert(unit: CurrentUnit) = internalMotor.getCurrentAlert(unit)

    fun setCurrentAlert(current: Double, unit: CurrentUnit) = internalMotor.setCurrentAlert(current, unit)

    fun setPIDFCoefficients(mode: RunMode, pidfCoefficients: PIDFCoefficients) = internalMotor.setPIDFCoefficients(mode, pidfCoefficients)

    fun getPIDFCoefficients(mode: RunMode): PIDFCoefficients = internalMotor.getPIDFCoefficients(mode)

    fun setVelocityPIDFCoefficients(p: Double, i: Double, d: Double, f: Double) = internalMotor.setVelocityPIDFCoefficients(p, i, d, f)

    fun setPositionPIDFCoefficients(p: Double) = internalMotor.setPositionPIDFCoefficients(p)

    init {
        mode = RunMode.STOP_AND_RESET_ENCODER
        mode = RunMode.RUN_WITHOUT_ENCODER
        zeroPowerBehavior = ZeroPowerBehavior.BRAKE
        direction = motorDirection
        internalMotor.power = 0.0
    }
}