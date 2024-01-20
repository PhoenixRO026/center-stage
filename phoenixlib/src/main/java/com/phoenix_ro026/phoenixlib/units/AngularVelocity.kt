package com.phoenix_ro026.phoenixlib.units

class AngularVelocity<out T: Angle, U: Duration>(val angle: T, val duration: U) {
    fun <V: Angle, W: Duration> to(unit: AngularVelocity<V, W>): AngularVelocity<V, W> {
        val angleVal = angleAsUnit(unit)
        val durationVal = unit.duration
        return angleVal.divT(durationVal)
    }
    fun <V: Angle, W: Duration> angleAsUnit(unit: AngularVelocity<V, W>): V {
        //2 / 1 = 2    1 meter per 2 seconds = 0.5 meter per 1 second
        val durationRatio = duration.to(unit.duration).value / unit.duration.value
        return angle.to(unit.angle).divT(durationRatio)
    }

    fun <V: Angle, W: Duration> durationAsUnit(unit: AngularVelocity<V, W>): W {
        //2 / 1 = 2    1 meter per 2 seconds = 0.5 meter per 1 second
        val angleRatio = angle.to(unit.angle).value / unit.angle.value
        return duration.to(unit.duration).divT(angleRatio)
    }

    operator fun <V: Angle, W: Duration> plus(other: AngularVelocity<V, W>) = AngularVelocity(angle + other.angleAsUnit(this), duration)
    operator fun <V: Angle, W: Duration> minus(other: AngularVelocity<V, W>) = AngularVelocity(angle - other.angleAsUnit(this), duration)
    operator fun unaryMinus() = AngularVelocity(-angle, duration)
    operator fun times(other: Number) = AngularVelocity(angle * other, duration)
    operator fun div(other: Number) = AngularVelocity(angle / other , duration)
    operator fun <V: Duration> times(other: V) = angleAsUnit(AngularVelocity(angle, other))
    operator fun <V: Angle> div(other: V) = durationAsUnit(AngularVelocity(other, duration))

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