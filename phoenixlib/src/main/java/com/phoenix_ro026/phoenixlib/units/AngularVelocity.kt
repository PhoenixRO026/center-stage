package com.phoenix_ro026.phoenixlib.units

class AngularVelocity(val angle: Angle, val duration: Duration) {
    fun to(unit: AngularVelocity): AngularVelocity {
        val angleVal = angleAsUnit(unit)
        val durationVal = unit.duration
        return angleVal / durationVal
    }
    fun angleAsUnit(unit: AngularVelocity): Angle {
        //2 / 1 = 2    1 meter per 2 seconds = 0.5 meter per 1 second
        val durationRatio = duration.to(unit.duration).value / unit.duration.value
        return angle.to(unit.angle) / durationRatio
    }

    fun durationAsUnit(unit: AngularVelocity): Duration {
        //2 / 1 = 2    1 meter per 2 seconds = 0.5 meter per 1 second
        val angleRatio = angle.to(unit.angle).value / unit.angle.value
        return duration.to(unit.duration) / angleRatio
    }

    operator fun plus(other: AngularVelocity) = AngularVelocity(angle + other.angleAsUnit(this), duration)
    operator fun minus(other: AngularVelocity) = AngularVelocity(angle - other.angleAsUnit(this), duration)
    operator fun unaryMinus() = AngularVelocity(-angle, duration)
    operator fun times(other: Number) = AngularVelocity(angle * other, duration)
    operator fun div(other: Number) = AngularVelocity(angle / other , duration)
    operator fun times(other: Duration) = angleAsUnit(AngularVelocity(angle, other))
    operator fun div(other: Angle) = durationAsUnit(AngularVelocity(other, duration))

    override fun toString(): String = "$angle per " + if (duration.value == 1.0) duration.unitToString() else "$duration"
}

@JvmField
val RPM = 1.rpm
val Number.rpm get() = rev / MIN
fun rpm(number: Double) = number.rpm
@JvmField
val RAD_SEC = 1.radSec
val Number.radSec get() = rad / SEC
fun radSec(number: Double) = number.radSec
@JvmField
val DEG_SEC = 1.degSec
val Number.degSec get() = deg / SEC
fun degSec(number: Double) = number.degSec