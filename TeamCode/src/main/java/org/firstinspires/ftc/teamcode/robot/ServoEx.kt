package org.firstinspires.ftc.teamcode.robot

import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.PwmControl.PwmRange
import com.qualcomm.robotcore.hardware.Servo
import com.qualcomm.robotcore.hardware.Servo.Direction
import com.qualcomm.robotcore.hardware.ServoControllerEx
import com.qualcomm.robotcore.hardware.ServoImplEx
import org.firstinspires.ftc.teamcode.lib.units.Angle
import org.firstinspires.ftc.teamcode.lib.units.deg
import kotlin.math.abs

@Suppress("unused", "MemberVisibilityCanBePrivate")
class ServoEx @JvmOverloads constructor(
    servo: Servo,
    private val changeTreshold: Double = 0.01,
    positionRange: ClosedRange<Double> = 0.0..1.0,
    private val maxAngle: Angle = 180.deg
) {
    companion object {
        const val minPwm = 500.0
        const val maxPwm = 2500.0
        const val maxPwmRange = maxPwm - minPwm

        @JvmOverloads
        fun axonMax180(servo: Servo, changeTreshold: Double = 0.01, range: ClosedRange<Double> = 0.0..1.0) = ServoEx(
            servo,
            changeTreshold,
            range,
            180.deg
        )

        @JvmOverloads
        fun HardwareMap.axonMax180(deviceName: String, changeTreshold: Double = 0.01, range: ClosedRange<Double> = 0.0..1.0) =
            axonMax180(
                get(Servo::class.java, deviceName),
                changeTreshold,
                range
            )

        @JvmOverloads
        fun axonMax355(servo: Servo, changeTreshold: Double = 0.01, range: ClosedRange<Double> = 0.0..1.0) = ServoEx(
            servo,
            changeTreshold,
            range,
            355.deg
        )

        @JvmOverloads
        fun HardwareMap.axonMax355(deviceName: String, changeTreshold: Double = 0.01, range: ClosedRange<Double> = 0.0..1.0) =
            axonMax355(
                get(Servo::class.java, deviceName),
                changeTreshold,
                range
            )
    }

    private val positionRange: ClosedRange<Double> = (positionRange.start.coerceIn(0.0, positionRange.endInclusive.coerceAtMost(1.0)))..(positionRange.endInclusive.coerceIn(positionRange.start.coerceAtLeast(0.0), 1.0))

    var cachedPosition: Double = 0.0
        private set

    var realPosition: Double = 0.0
        private set(value) {
            val newPosition = value.coerceIn(0.0, 1.0)
            val overChangeThreshold = abs(newPosition - cachedPosition) >= changeTreshold
            val targetingLimitPosition = (newPosition >= positionRange.endInclusive && cachedPosition < positionRange.endInclusive) || (newPosition <= positionRange.start && cachedPosition > positionRange.start)
            val targetingMaxPosition = (newPosition >= 1.0 && cachedPosition < 1.0) || (newPosition <= 0.0 && cachedPosition > 0.0)
            if (overChangeThreshold || targetingLimitPosition || targetingMaxPosition) {
                internalServo.position = newPosition
                cachedPosition = newPosition
            }
            field = newPosition
        }

    val internalServo: ServoImplEx = servo as ServoImplEx

    val controller: ServoControllerEx = internalServo.controller as ServoControllerEx

    val portNumber: Int = internalServo.portNumber

    var position: Double
        get() = realPosition.reverseScale(positionRange)
        set(value) {
            realPosition = value.coerceIn(0.0, 1.0).scaleTo(positionRange)
        }

    val servoRange get() = ((pwnRange.usPulseLower - minPwm) / maxPwmRange)..((pwnRange.usPulseUpper - minPwm) / maxPwmRange)

    var angle: Angle
        get() = maxAngle * realPosition.scaleTo(servoRange)
        set(value) {
            realPosition = (value.coerceIn(0.deg, maxAngle) / maxAngle).reverseScale(servoRange)
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

    fun Double.scaleTo(range: ClosedRange<Double>): Double {
        return range.start + this * (range.endInclusive - range.start)
    }

    fun Double.reverseScale(range: ClosedRange<Double>): Double {
        return (this - range.start) / (range.endInclusive - range.start)
    }

}