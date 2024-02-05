package org.firstinspires.ftc.teamcode.robot.hardware

import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.PwmControl.PwmRange
import com.qualcomm.robotcore.hardware.Servo
import com.qualcomm.robotcore.hardware.Servo.Direction
import com.qualcomm.robotcore.hardware.ServoControllerEx
import com.qualcomm.robotcore.hardware.ServoImplEx
import org.firstinspires.ftc.teamcode.lib.units.Angle
import org.firstinspires.ftc.teamcode.lib.units.Time
import org.firstinspires.ftc.teamcode.lib.units.deg
import kotlin.math.abs
import kotlin.math.sign

@Suppress("unused", "MemberVisibilityCanBePrivate")
class ServoEx @JvmOverloads constructor(
    servo: Servo,
    private val maxAngle: Angle = 180.deg,
    positionRange: ClosedRange<Double> = 0.0..1.0,
    servoDirection: Direction = servo.direction,
    private val changeTreshold: Double = 0.01,
    private val speed: Double = 0.1,
    private val onPositionUpdate: () -> Unit = {},
) {
    companion object {
        const val minPwm = 500.0
        const val maxPwm = 2500.0
        const val maxPwmRange = maxPwm - minPwm

        @JvmOverloads
        fun axonMax180(servo: Servo, range: ClosedRange<Double> = 0.0..1.0, changeTreshold: Double = 0.01) = ServoEx(
            servo = servo,
            maxAngle = 180.deg,
            positionRange = range,
            changeTreshold = changeTreshold
        )

        @JvmOverloads
        fun HardwareMap.axonMax180(deviceName: String, range: ClosedRange<Double> = 0.0..1.0, changeTreshold: Double = 0.01) =
            axonMax180(
                servo = get(Servo::class.java, deviceName),
                range = range,
                changeTreshold = changeTreshold
            )

        @JvmOverloads
        fun axonMax355(servo: Servo, range: ClosedRange<Double> = 0.0..1.0, changeTreshold: Double = 0.01) = ServoEx(
            servo = servo,
            maxAngle = 355.deg,
            positionRange = range,
            changeTreshold = changeTreshold
        )

        @JvmOverloads
        fun HardwareMap.axonMax355(deviceName: String, range: ClosedRange<Double> = 0.0..1.0, changeTreshold: Double = 0.01) =
            axonMax355(
                servo = get(Servo::class.java, deviceName),
                range = range,
                changeTreshold = changeTreshold
            )
    }

    private val positionRange: ClosedRange<Double> = (positionRange.start.coerceIn(0.0, positionRange.endInclusive.coerceAtMost(1.0)))..(positionRange.endInclusive.coerceIn(positionRange.start.coerceAtLeast(0.0), 1.0))

    var cachedPosition: Double = 0.0
        private set(value) {
            if (field != value) {
                internalServo.position = value
                onPositionUpdate()
            }
            field = value
        }

    var realTargetPosition: Double = 0.0
        private set

    var realPosition: Double = 0.0
        private set(value) {
            val newPosition = value.coerceIn(0.0, 1.0)
            val overChangeThreshold = abs(newPosition - cachedPosition) >= changeTreshold
            val targetingLimitPosition = (newPosition >= positionRange.endInclusive && cachedPosition < positionRange.endInclusive) || (newPosition <= positionRange.start && cachedPosition > positionRange.start)
            val targetingMaxPosition = (newPosition >= 1.0 && cachedPosition < 1.0) || (newPosition <= 0.0 && cachedPosition > 0.0)
            if (overChangeThreshold || targetingLimitPosition || targetingMaxPosition) {
                cachedPosition = newPosition
            }
            field = newPosition
        }

    val internalServo: ServoImplEx = servo as ServoImplEx

    val controller: ServoControllerEx = internalServo.controller as ServoControllerEx

    val portNumber: Int = internalServo.portNumber

    val isBusy get() = realPosition != realTargetPosition

    var position: Double
        get() = realPosition.reverseScale(positionRange).coerceIn(0.0, 1.0)
        set(value) {
            realPosition = value.coerceIn(0.0, 1.0).scaleTo(positionRange)
            realTargetPosition = realPosition
        }

    var targetPosition: Double
        get() = realTargetPosition.reverseScale(positionRange).coerceIn(0.0, 1.0)
        set(value) {
            realTargetPosition = value.coerceIn(0.0, 1.0).scaleTo(positionRange)
        }

    val servoRange get() = ((pwnRange.usPulseLower - minPwm) / maxPwmRange)..((pwnRange.usPulseUpper - minPwm) / maxPwmRange)

    var angle: Angle
        get() = maxAngle * realPosition.scaleTo(servoRange)
        set(value) {
            realPosition = (value.coerceIn(0.deg, maxAngle) / maxAngle).reverseScale(servoRange)
            realTargetPosition = realPosition
        }

    var targetAngle: Angle
        get() = maxAngle * realTargetPosition.scaleTo(servoRange)
        set(value) {
            realTargetPosition = (value.coerceIn(0.deg, maxAngle) / maxAngle).reverseScale(servoRange)
        }

    var pwnRange: PwmRange = internalServo.pwmRange
        set(value) {
            if (value != field) {
                internalServo.pwmRange = value
            }
            field = value
        }

    var disabled: Boolean = !internalServo.isPwmEnabled
        set(value) {
            if (field != value) {
                if (value) {
                    internalServo.setPwmDisable()
                } else {
                    internalServo.setPwmEnable()
                }
            }
            field = value
        }

    var direction: Direction = internalServo.direction
        set(value) {
            if (field != value) {
                internalServo.direction = value
            }
            field = value
        }

    fun update(deltaTime: Time) {
        val error = realTargetPosition - realPosition
        if (error == 0.0) return
        val step = speed * deltaTime.s
        if (abs(error) < step) {
            realPosition += error
            if (realPosition != cachedPosition) {
                cachedPosition = realTargetPosition
            }
        } else {
            realPosition += sign(error) * step
        }
    }

    init {
        direction = servoDirection
    }
}