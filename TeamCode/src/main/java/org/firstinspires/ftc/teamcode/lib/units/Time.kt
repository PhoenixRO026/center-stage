package org.firstinspires.ftc.teamcode.lib.units

class Time(val s: Double) {
    val ms get() = s * 1000.0
    val min get() = s / 60.0

    operator fun plus(other: Time) = Time(s + other.s)
    operator fun minus(other: Time) = Time(s - other.s)
    operator fun times(scalar: Number) = Time(s * scalar.toDouble())
    operator fun div(scalar: Number) = Time(s / scalar.toDouble())
    operator fun div(other: Time) = s / other.s
    operator fun unaryMinus() = Time(-s)

    override fun toString() = "$s " + if (s == 1.0) "second" else "seconds"
}

val Number.s get() = Time(toDouble())
val Number.ms get() = Time(toDouble() / 1000.0)
val Number.min get() = Time(toDouble() * 60.0)

fun s(number: Double) = number.s
fun ms(number: Double) = number.ms
fun min(number: Double) = number.min

@JvmField val s = 1.s
@JvmField val ms = 1.ms
@JvmField val min = 1.min