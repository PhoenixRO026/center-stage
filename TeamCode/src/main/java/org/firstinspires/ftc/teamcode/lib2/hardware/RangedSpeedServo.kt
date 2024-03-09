package org.firstinspires.ftc.teamcode.lib2.hardware

import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.Servo.Direction
import org.firstinspires.ftc.teamcode.lib.units.Time

class RangedSpeedServo @JvmOverloads constructor(
    @JvmField val innerServo: SpeedServo,
    @JvmField val range: ClosedRange<Double> = 0.0..1.0
) {
    companion object {
        fun HardwareMap.rangedSpeedServo(
            deviceName: String,
            direction: Direction = Direction.FORWARD,
            speed: Double = SpeedServo.defaultSpeed,
            range: ClosedRange<Double> = 0.0..1.0
        ) = RangedSpeedServo(this, deviceName, direction, speed, range)

        fun HardwareMap.rangedSpeedCachedServo(
            deviceName: String,
            direction: Direction = Direction.FORWARD,
            cachingThreshold: Double = CachedServo.defaultCachingThreshold,
            speed: Double = SpeedServo.defaultSpeed,
            range: ClosedRange<Double> = 0.0..1.0
        ) = CachedServo(this, deviceName, direction, cachingThreshold).rangedSpeed(speed, range)
    }

    @JvmOverloads constructor(
        hardwareMap: HardwareMap,
        deviceName: String,
        direction: Direction = Direction.FORWARD,
        speed: Double = SpeedServo.defaultSpeed,
        range: ClosedRange<Double> = 0.0..1.0
    ) : this(SpeedServo(hardwareMap, deviceName, direction, speed), range)

    @JvmOverloads constructor(
        servo: ServoEx,
        speed: Double = SpeedServo.defaultSpeed,
        range: ClosedRange<Double> = 0.0..1.0
    ) : this(SpeedServo(servo, speed), range)

    var unscaledPosition: Double by innerServo::position

    var unscaledTargetPosition: Double by innerServo::targetPosition

    var position: Double
        get() = unscaledPosition.reverseScale(range).coerceIn(0.0, 1.0)
        set(value) {
            unscaledPosition = value.coerceIn(0.0, 1.0).scaleTo(range)
        }

    var targetPosition: Double
        get() = unscaledTargetPosition.reverseScale(range).coerceIn(0.0, 1.0)
        set(value) {
            unscaledTargetPosition = value.coerceIn(0.0, 1.0).scaleTo(range)
        }

    fun update(deltaTime: Time) = innerServo.update(deltaTime)
}