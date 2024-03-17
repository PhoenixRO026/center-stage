package org.firstinspires.ftc.teamcode.lib.hardware.servo

import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.Servo
import com.qualcomm.robotcore.hardware.Servo.Direction
import org.firstinspires.ftc.teamcode.lib.units.DeltaTime
import kotlin.math.abs
import kotlin.math.sign

open class SpeedServo @JvmOverloads constructor(
    servo: Servo,
    direction: Direction = Direction.FORWARD,
    @JvmField val speed: Double = defaultSpeed
) : SimpleServo(servo, direction) {
    companion object {
        const val defaultSpeed = 0.1

        fun HardwareMap.speedServo(
            deviceName: String,
            direction: Direction = Direction.FORWARD,
            speed: Double = defaultSpeed
        ) = SpeedServo(this, deviceName, direction, speed)
    }

    @JvmOverloads constructor(
        hardwareMap: HardwareMap,
        deviceName: String,
        direction: Direction = Direction.FORWARD,
        speed: Double = defaultSpeed
    ) : this(hardwareMap.get(Servo::class.java, deviceName), direction, speed)

    private val deltaTime = DeltaTime()

    override var position: Double
        get() = super.position
        set(value) {
            super.position = value
            targetPosition = value.coerceIn(0.0, 1.0)
        }

    open var targetPosition: Double = super.position

    open fun update() {
        val error = targetPosition - position
        if (error == 0.0) return
        val step = speed * deltaTime.calculateDeltaTime().s
        position += if (abs(error) < step) {
            error
        } else {
            error.sign * step
        }
    }
}