package org.firstinspires.ftc.teamcode.lib.hardware.newer

import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.PwmControl.PwmRange
import com.qualcomm.robotcore.hardware.Servo
import com.qualcomm.robotcore.hardware.Servo.Direction
import com.qualcomm.robotcore.hardware.ServoImplEx
import org.firstinspires.ftc.teamcode.lib.hardware.reverseScale
import org.firstinspires.ftc.teamcode.lib.hardware.scaleTo
import kotlin.math.abs

class Servo @JvmOverloads constructor(
    servo: Servo,
    direction: Direction = servo.direction,
    val range: ClosedRange<Double> = 0.0..1.0,
    private val cachingThreshold: Double = 0.01
) {
    companion object {
        fun HardwareMap.servo(
            deviceName: String,
            direction: Direction = Direction.FORWARD,
            range: ClosedRange<Double> = 0.0..1.0,
            cachingThreshold: Double = 0.01
        ) = Servo(this, deviceName, direction, range, cachingThreshold)
    }

    @JvmOverloads constructor(
        hardwareMap: HardwareMap,
        deviceName: String,
        direction: Direction = hardwareMap.get(Servo::class.java, deviceName).direction,
        range: ClosedRange<Double> = 0.0..1.0,
        cachingThreshold: Double = 0.01
    ) : this(
        hardwareMap.get(Servo::class.java, deviceName),
        direction,
        range,
        cachingThreshold
    )

    private var innerServoPosition = 0.0
        set(value) {
            val newValue = value.coerceIn(0.0, 1.0)
            if (field != newValue) {
                innerServo.position = newValue
            }
            field = newValue
        }

    val innerServo = servo as ServoImplEx

    var unscaledPosition = 0.0
        set(value) {
            val newValue = value.coerceIn(0.0, 1.0)
            if (shouldChangePosition(newValue)) {
                innerServoPosition = newValue
            }
            field = newValue
        }

    var position
        get() = unscaledPosition.reverseScale(range).coerceIn(0.0, 1.0)
        set(value) {
            unscaledPosition = value.coerceIn(0.0, 1.0).scaleTo(range)
        }

    var pwmRange: PwmRange = innerServo.pwmRange
        set(value) {
            if (field != value) {
                innerServo.pwmRange = value
            }
            field = value
        }

    var direction = direction
        set(value) {
            if (field != value) {
                innerServo.direction = value
            }
            field = value
        }

    init {
        innerServo.direction = direction
    }

    fun setUnscaledPositionResult(position: Double): Boolean {
        val result = shouldChangePosition(position)
        unscaledPosition = position
        return result
    }

    fun setPositionResult(position: Double): Boolean {
        val pos = position.coerceIn(0.0, 1.0).scaleTo(range)
        val result = shouldChangePosition(pos)
        unscaledPosition = pos
        return result
    }

    fun targetServo(speed: Double = 0.1) = TargetServo(this, speed)

    fun shouldChangePosition(pos: Double): Boolean {
        val newPosition = pos.coerceIn(0.0, 1.0)
        val overChangeThreshold = abs(newPosition - innerServoPosition) >= cachingThreshold
        val targetingLimitPosition = (newPosition >= range.endInclusive && innerServoPosition < range.endInclusive) || (newPosition <= range.start && innerServoPosition > range.start)
        val targetingMaxPosition = (newPosition >= 1.0 && innerServoPosition < 1.0) || (newPosition <= 0.0 && innerServoPosition > 0.0)
        return overChangeThreshold || targetingLimitPosition || targetingMaxPosition
    }

    fun forcePositionUpdate() {
        innerServoPosition = unscaledPosition
    }
}