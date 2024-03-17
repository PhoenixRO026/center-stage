package org.firstinspires.ftc.teamcode.lib.hardware.servo

import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.Servo
import org.firstinspires.ftc.teamcode.lib.hardware.servo.SpeedServo.Companion.defaultSpeed
import kotlin.math.abs
import kotlin.math.sign

class RangedSpeedCachedServo(
    servo: Servo,
    direction: Servo.Direction = Servo.Direction.FORWARD,
    cachingThreshold: Double = defaultCachingThreshold,
    speed: Double = defaultSpeed,
    @JvmField val range: ClosedRange<Double> = 0.0..1.0,
    private val coupledServos: List<SimpleServo> = emptyList()
) : SpeedCachedServo(servo, direction, cachingThreshold, speed) {
    companion object {
        fun HardwareMap.rangedSpeedCachedServo(
            deviceName: String,
            direction: Servo.Direction = Servo.Direction.FORWARD,
            cachingThreshold: Double = defaultCachingThreshold,
            speed: Double = defaultSpeed,
            range: ClosedRange<Double> = 0.0..1.0,
            coupledServos: List<SimpleServo> = emptyList()
        ) = RangedSpeedCachedServo(this, deviceName, direction, cachingThreshold, speed, range, coupledServos)
    }

    @JvmOverloads constructor(
        hardwareMap: HardwareMap,
        deviceName: String,
        direction: Servo.Direction = Servo.Direction.FORWARD,
        cachingThreshold: Double = defaultCachingThreshold,
        speed: Double = defaultSpeed,
        range: ClosedRange<Double> = 0.0..1.0,
        coupledServos: List<SimpleServo> = emptyList()
    ) : this(hardwareMap.get(Servo::class.java, deviceName), direction, cachingThreshold, speed, range, coupledServos)

    val unscaledCachedPosition
        get() = super.cachedPosition

    override val cachedPosition: Double
        get() = unscaledCachedPosition.reverseScale(range).coerceIn(0.0, 1.0)

    var unscaledPosition
        get() = super.position
        set(value) {
            super.position = value
            coupledServos.forEach {
                it.position = position
            }
        }

    override var position: Double
        get() = unscaledPosition.reverseScale(range).coerceIn(0.0, 1.0)
        set(value) {
            unscaledPosition = value.coerceIn(0.0, 1.0).scaleTo(range)
        }

    var unscaledTargetPosition
        get() = super.targetPosition
        set(value) {
            super.targetPosition = value
        }

    override var targetPosition: Double
        get() = unscaledTargetPosition.reverseScale(range).coerceIn(0.0, 1.0)
        set(value) {
            unscaledTargetPosition = value.coerceIn(0.0, 1.0).scaleTo(range)
        }

    fun setUnscaledPositionResult(position: Double): Boolean = super.setPositionResult(position)

    override fun setPositionResult(position: Double): Boolean = setUnscaledPositionResult(position.coerceIn(0.0, 1.0).scaleTo(range))

    override fun update() {
        val error = unscaledTargetPosition - unscaledPosition
        if (error == 0.0) return
        val step = speed * deltaTime.calculateDeltaTime().s
        super.position += if (abs(error) < step) {
            error
        } else {
            error.sign * step
        }
        coupledServos.forEach {
            it.position = position
        }
    }
}