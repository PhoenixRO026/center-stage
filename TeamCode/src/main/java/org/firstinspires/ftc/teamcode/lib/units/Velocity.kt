@file:Suppress("unused")

package org.firstinspires.ftc.teamcode.lib.units

data class Velocity(val distance: Distance, val time: Time) {
    val msec get() = this / (m / s)
    val cmsec get() = this / (cm / s)
    val mmsec get() = this / (mm / s)
    val inchsec get() = this / (inch / s)

    operator fun plus(other: Velocity) = Velocity(distance + other.distance * (time / other.time), time)
    operator fun minus(other: Velocity) = Velocity(distance - other.distance * (time / other.time), time)
    operator fun times(scalar: Number) = Velocity(distance * scalar, time)
    operator fun times(other: Time) = distance * (other / time)
    operator fun div(scalar: Number) = Velocity(distance / scalar, time)
    operator fun div(other: Distance) = time * (other / distance)
    operator fun div(other: Velocity) = (distance / other.distance) / (time / other.time)
    operator fun unaryMinus() = Velocity(-distance, time)

    override fun toString() = "$distance per " + if (time.s == 1.0) "second" else "$time"
}

val Number.msec get() = this.m / s
val Number.cmsec get() = this.cm / s
val Number.mmsec get() = this.mm / s
val Number.inchsec get() = this.inch / s

fun msec(number: Double) = number.msec
fun cmsec(number: Double) = number.cmsec
fun mmsec(number: Double) = number.mmsec
fun inchsec(number: Double) = number.inchsec

@JvmField val msec = 1.msec
@JvmField val cmsec = 1.cmsec
@JvmField val mmsec = 1.mmsec
@JvmField val inchsec = 1.inchsec