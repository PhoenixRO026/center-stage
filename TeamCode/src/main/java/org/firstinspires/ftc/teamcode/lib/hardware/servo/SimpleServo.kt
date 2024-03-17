package org.firstinspires.ftc.teamcode.lib.hardware.servo

import com.qualcomm.hardware.lynx.LynxServoController
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.PwmControl
import com.qualcomm.robotcore.hardware.Servo
import com.qualcomm.robotcore.hardware.ServoImplEx

open class SimpleServo @JvmOverloads constructor(
    servo: Servo,
    direction: Servo.Direction = Servo.Direction.FORWARD
)  {
    companion object {
        fun HardwareMap.simpleServo(
            deviceName: String,
            direction: Servo.Direction = Servo.Direction.FORWARD
        ) = SimpleServo(this, deviceName, direction)
    }

    @JvmOverloads constructor(
        hardwareMap: HardwareMap,
        deviceName: String,
        direction: Servo.Direction = Servo.Direction.FORWARD
    ) : this(
        hardwareMap.get(Servo::class.java, deviceName),
        direction
    )

    protected var hasPositionBeenSet = false

    @JvmField val innerServo = servo as ServoImplEx

    @JvmField val controller = innerServo.controller as LynxServoController

    open var position: Double = 0.0
        set(value) {
            val newValue = value.coerceIn(0.0, 1.0)
            if (field != newValue || !hasPositionBeenSet) {
                hasPositionBeenSet = true
                innerServo.position = newValue
            }
            field = newValue
        }

    var direction: Servo.Direction = direction
        set(value) {
            if (field != value) {
                innerServo.direction = value
            }
            field = value
        }

    var pwmRange: PwmControl.PwmRange = innerServo.pwmRange
        set(value) {
            if (field != value) {
                innerServo.pwmRange = value
            }
            field = value
        }

    init {
        innerServo.resetDeviceConfigurationForOpMode()
        innerServo.direction = direction
    }
}