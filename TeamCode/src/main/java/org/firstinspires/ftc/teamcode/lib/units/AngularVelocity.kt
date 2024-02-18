@file:Suppress("unused")

package org.firstinspires.ftc.teamcode.lib.units

data class AngularVelocity(val angle: Angle, val time: Time) {
    val radsec get() = this / (rad / s)
    val degsec get() = this / (deg / s)
    val revsec get() = this / (rev / s)
    val rpm get() = this / (rev / min)

    operator fun plus(other: AngularVelocity) = AngularVelocity(angle + other.angle * (time / other.time), time)
    operator fun minus(other: AngularVelocity) = AngularVelocity(angle - other.angle * (time / other.time), time)
    operator fun times(scalar: Number) = AngularVelocity(angle * scalar, time)
    operator fun times(other: Time) = angle * (other / time)
    operator fun div(scalar: Number) = AngularVelocity(angle / scalar, time)
    operator fun div(other: Angle) = time * (other / angle)
    operator fun div(other: AngularVelocity) = (angle / other.angle) / (time / other.time)
    operator fun unaryMinus() = AngularVelocity(-angle, time)

    override fun toString() = "$angle per " + if (time.s == 1.0) "second" else "$time"
}

val Number.radsec get() = this.rad / s
val Number.degsec get() = this.deg / s
val Number.revsec get() = this.rev / s
val Number.rpm get() = this.rev / min

fun radsec(number: Double) = number.radsec
fun degsec(number: Double) = number.degsec
fun revsec(number: Double) = number.revsec
fun rpm(number: Double) = number.rpm

@JvmField val radsec = 1.radsec
@JvmField val degsec = 1.degsec
@JvmField val revsec = 1.revsec
@JvmField val rpm = 1.rpm