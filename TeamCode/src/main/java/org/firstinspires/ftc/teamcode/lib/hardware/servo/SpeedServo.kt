package org.firstinspires.ftc.teamcode.lib.hardware.servo

import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.Servo
import com.qualcomm.robotcore.hardware.Servo.Direction
import com.phoenix.phoenixlib.units.DeltaTime
import kotlin.math.abs
import kotlin.math.sign

open class SpeedServo @JvmOverloads constructor(
    servo: Servo,
    direction: Direction = Direction.FORWARD,
    @JvmField val speed: Double = defaultSpeed,
    private val coupledServos: List<SimpleServo> = emptyList()
) : SimpleServo(servo, direction) {
    companion object {
        const val defaultSpeed = 0.1

        fun HardwareMap.speedServo(
            deviceName: String,
            direction: Direction = Direction.FORWARD,
            speed: Double = defaultSpeed,
            coupledServos: List<SimpleServo> = emptyList()
        ) = SpeedServo(this, deviceName, direction, speed, coupledServos)
    }

    @JvmOverloads constructor(
        hardwareMap: HardwareMap,
        deviceName: String,
        direction: Direction = Direction.FORWARD,
        speed: Double = defaultSpeed,
        coupledServos: List<SimpleServo> = emptyList()
    ) : this(hardwareMap.get(Servo::class.java, deviceName), direction, speed, coupledServos)

    protected val deltaTime = DeltaTime()

    private var innerPosition: Double
        get() = super.position
        set(value) {
            super.position = value
            coupledServos.forEach {
                it.position = position
            }
            innerTargetPosition = value.coerceIn(0.0, 1.0)
        }

    override var position: Double by ::innerPosition

    private var innerTargetPosition = super.position

    open var targetPosition: Double by :: innerTargetPosition

    val isBusy: Boolean get() = innerTargetPosition != innerPosition

    fun update() {
        val error = innerTargetPosition - innerPosition
        val step = speed * deltaTime.calculateDeltaTime().s
        if (error == 0.0) return
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