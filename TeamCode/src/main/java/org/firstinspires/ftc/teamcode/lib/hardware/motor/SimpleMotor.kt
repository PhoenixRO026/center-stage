package org.firstinspires.ftc.teamcode.lib.hardware.motor

import com.qualcomm.hardware.lynx.LynxDcMotorController
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorImplEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.DcMotorSimple.Direction
import com.qualcomm.robotcore.hardware.HardwareMap

open class SimpleMotor @JvmOverloads constructor(
    dcMotor: DcMotorSimple,
    direction: Direction = Direction.FORWARD
) {
    companion object {
        fun HardwareMap.simpleMotor(
            deviceName: String,
            direction: Direction = Direction.FORWARD
        ) = SimpleMotor(this, deviceName, direction)
    }

    @JvmOverloads constructor(
        hardwareMap: HardwareMap,
        deviceName: String,
        direction: Direction = Direction.FORWARD
    ) : this(hardwareMap.get(DcMotorSimple::class.java, deviceName), direction)

    @JvmField val innerMotor = dcMotor as DcMotorImplEx
    @JvmField val controller = innerMotor.controller as LynxDcMotorController

    private var positionOffset: Int = 0

    open var power: Double = 0.0
        set(value) {
            val newValue = value.coerceIn(-1.0, 1.0)
            if (field != newValue) {
                innerMotor.power = newValue
            }
            field = newValue
        }

    var direction = direction
        set(value) {
            if (field != value) {
                innerMotor.direction = value
            }
            field = value
        }

    var mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        set(value) {
            if (field != value) {
                innerMotor.mode = value
            }
            field = value
        }

    val positionTicks: Int get() = innerMotor.currentPosition - positionOffset

    val velocityTicks: Double get() = innerMotor.velocity

    var targetPositionTicks: Int = 0
        set(value) {
            if (field != value) {
                innerMotor.targetPosition = value + positionOffset
            }
            field = value
        }

    var zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        set(value) {
            if (field != value) {
                innerMotor.zeroPowerBehavior = value
            }
            field = value
        }

    fun resetPosition() {
        positionOffset = innerMotor.currentPosition
        innerMotor.targetPosition = targetPositionTicks + positionOffset
    }

    init {
        innerMotor.resetDeviceConfigurationForOpMode()
        innerMotor.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        innerMotor.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        innerMotor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        innerMotor.direction = direction
        innerMotor.power = 0.0
    }
}