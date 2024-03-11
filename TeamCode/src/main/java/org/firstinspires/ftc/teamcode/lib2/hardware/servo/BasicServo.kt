package org.firstinspires.ftc.teamcode.lib2.hardware.servo

import com.qualcomm.hardware.lynx.LynxServoController
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.PwmControl.PwmRange
import com.qualcomm.robotcore.hardware.Servo
import com.qualcomm.robotcore.hardware.Servo.Direction
import com.qualcomm.robotcore.hardware.ServoImplEx

class BasicServo @JvmOverloads constructor(
    servo: Servo,
    direction: Direction = Direction.FORWARD
) : ServoEx {
    companion object {
        fun HardwareMap.basicServo(
            deviceName: String,
            direction: Direction = Direction.FORWARD
        ) = BasicServo(this, deviceName, direction)
    }

    @JvmOverloads constructor(
        hardwareMap: HardwareMap,
        deviceName: String,
        direction: Direction = Direction.FORWARD
    ) : this(
        hardwareMap.get(Servo::class.java, deviceName),
        direction
    )

    private var hasPositionBeenSet = false

    @JvmField val innerServo = servo as ServoImplEx

    @JvmField val controller = innerServo.controller as LynxServoController

    override var position: Double = 0.0
        set(value) {
            val newValue = value.coerceIn(0.0, 1.0)
            if (field != newValue || !hasPositionBeenSet) {
                hasPositionBeenSet = true
                innerServo.position = newValue
            }
            field = newValue
        }

    var direction: Direction = direction
        set(value) {
            if (field != value) {
                innerServo.direction = value
            }
            field = value
        }

    var pwmRange: PwmRange = innerServo.pwmRange
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

    fun cached(cachingThreshold: Double = CachedServo.defaultCachingThreshold) = CachedServo(this, cachingThreshold)
}