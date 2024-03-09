package org.firstinspires.ftc.teamcode.lib2.hardware

import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.Servo.Direction

class RangedServo @JvmOverloads constructor(
    @JvmField val innerServo: ServoEx,
    @JvmField val range: ClosedRange<Double> = 0.0..1.0
) : ServoEx {

    companion object {
        fun HardwareMap.rangedServo(
            deviceName: String,
            direction: Direction = Direction.FORWARD,
            range: ClosedRange<Double> = 0.0..1.0
        ) = RangedServo(this, deviceName, direction, range)

        fun HardwareMap.rangedCachedServo(
            deviceName: String,
            direction: Direction = Direction.FORWARD,
            cachingThreshold: Double = CachedServo.defaultCachingThreshold,
            range: ClosedRange<Double> = 0.0..1.0
        ) = CachedServo(this, deviceName, direction, cachingThreshold).ranged(range)
    }

    @JvmOverloads constructor(
        hardwareMap: HardwareMap,
        deviceName: String,
        direction: Direction = Direction.FORWARD,
        range: ClosedRange<Double> = 0.0..1.0
    ) : this(BasicServo(hardwareMap, deviceName, direction), range)

    var unscaledPosition: Double by innerServo::position

    override var position: Double
        get() = unscaledPosition.reverseScale(range).coerceIn(0.0, 1.0)
        set(value) {
            unscaledPosition = value.coerceIn(0.0, 1.0).scaleTo(range)
        }
}