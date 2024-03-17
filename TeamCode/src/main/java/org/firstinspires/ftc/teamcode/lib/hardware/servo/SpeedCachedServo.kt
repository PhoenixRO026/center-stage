package org.firstinspires.ftc.teamcode.lib.hardware.servo

import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.Servo
import com.qualcomm.robotcore.hardware.Servo.Direction
import org.firstinspires.ftc.teamcode.lib.hardware.servo.SpeedServo.Companion.defaultSpeed
import org.firstinspires.ftc.teamcode.lib.units.DeltaTime
import kotlin.math.abs
import kotlin.math.sign

open class SpeedCachedServo(
    servo: Servo,
    direction: Direction = Direction.FORWARD,
    cachingThreshold: Double = defaultCachingThreshold,
    @JvmField val speed: Double = defaultSpeed
) : CachedServo(servo, direction, cachingThreshold) {

    companion object {
        fun HardwareMap.speedCachedServo(
            deviceName: String,
            direction: Direction = Direction.FORWARD,
            cachingThreshold: Double = defaultCachingThreshold,
            speed: Double = defaultSpeed
        ) = SpeedCachedServo(this, deviceName, direction, cachingThreshold, speed)
    }

    @JvmOverloads constructor(
        hardwareMap: HardwareMap,
        deviceName: String,
        direction: Direction = Direction.FORWARD,
        cachingThreshold: Double = defaultCachingThreshold,
        speed: Double = defaultSpeed
    ) : this(hardwareMap.get(Servo::class.java, deviceName), direction, cachingThreshold, speed)

    private val deltaTime = DeltaTime()

    override var position: Double
        get() = super.position
        set(value) {
            super.position = value
            targetPosition = value.coerceIn(0.0, 1.0)
        }

    open var targetPosition: Double = super.position

    override fun setPositionResult(position: Double): Boolean {
        val pos = position.coerceIn(0.0, 1.0)
        val result = shouldChangePosition(pos)
        this.position = pos
        return result
    }

    fun update() {
        val error = targetPosition - position
        if (error == 0.0) return
        val step = speed * deltaTime.calculateDeltaTime().s
        if (abs(error) < step) {
            position += error
            forceUpdate()
        } else {
            position += error.sign * step
        }
    }
}