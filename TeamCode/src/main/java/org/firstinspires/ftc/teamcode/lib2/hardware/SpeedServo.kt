package org.firstinspires.ftc.teamcode.lib2.hardware

import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.Servo.Direction
import org.firstinspires.ftc.teamcode.lib.units.Time
import kotlin.math.abs
import kotlin.math.sign

class SpeedServo @JvmOverloads constructor(
    @JvmField val innerServo: ServoEx,
    @JvmField val speed: Double = defaultSpeed
): ServoEx {
    companion object {
        const val defaultSpeed = 0.1

        fun HardwareMap.speedServo(
            deviceName: String,
            direction: Direction = Direction.FORWARD,
            speed: Double = defaultSpeed
        ) = SpeedServo(this, deviceName, direction, speed)

        fun HardwareMap.speedCachedServo(
            deviceName: String,
            direction: Direction = Direction.FORWARD,
            cachingThreshold: Double = CachedServo.defaultCachingThreshold,
            speed: Double = defaultSpeed
        ) = CachedServo(this, deviceName, direction, cachingThreshold).speed(speed)
    }

    @JvmOverloads constructor(
        hardwareMap: HardwareMap,
        deviceName: String,
        direction: Direction = Direction.FORWARD,
        speed: Double = defaultSpeed
    ) : this(BasicServo(hardwareMap, deviceName, direction), speed)

    private val cachedServo: CachedServo? = if (innerServo is CachedServo) {
        innerServo
    } else if (innerServo is RangedServo) {
        if (innerServo.innerServo is CachedServo) {
            innerServo.innerServo
        } else null
    } else null

    override var position: Double
        get() = innerServo.position
        set(value) {
            innerServo.position = value
            targetPosition = value.coerceIn(0.0, 1.0)
        }

    var targetPosition: Double = position

    fun update(deltaTime: Time) {
        val error = targetPosition - position
        if (error == 0.0) return
        val step = speed * deltaTime.s
        if (abs(error) < step) {
            position += error
            cachedServo?.forceUpdate()
        } else {
            position += error.sign * step
        }
    }

    fun rangedSpeed(range: ClosedRange<Double> = 0.0..1.0) = RangedSpeedServo(this, range)
}