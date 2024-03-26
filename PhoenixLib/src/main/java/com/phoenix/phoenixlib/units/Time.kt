@file:Suppress("unused")

package com.phoenix.phoenixlib.units

import com.acmerobotics.roadrunner.SleepAction

class Time(@JvmField var s: Double) {
    companion object {
        fun now() = System.currentTimeMillis().ms
    }

    val ms get() = s.sToMs()
    val min get() = s.sToMin()

    operator fun plus(other: Time) = Time(s + other.s)
    operator fun minus(other: Time) = Time(s - other.s)
    operator fun times(scalar: Number) = Time(s * scalar.toDouble())
    operator fun div(scalar: Number) = Time(s / scalar.toDouble())
    operator fun div(other: Time) = s / other.s
    operator fun unaryMinus() = Time(-s)
    operator fun compareTo(other: Time) = s.compareTo(other.s)

    override fun toString() = "$s " + if (s == 1.0) "second" else "seconds"
}

fun Number.sToMs() = toDouble() * 1000.0
fun Number.sToMin() = toDouble() / 60.0
fun Number.msToS() = toDouble() / 1000.0
fun Number.msToMin() = toDouble() / 60_000.0
fun Number.minToS() = toDouble() * 60.0
fun Number.minToMs() = toDouble() * 60_000.0

val Number.s get() = Time(toDouble())
val Number.ms get() = Time(msToS())
val Number.min get() = Time(minToS())

fun s(number: Double) = number.s
fun ms(number: Double) = number.ms
fun min(number: Double) = number.min

@JvmField val s = 1.s
@JvmField val ms = 1.ms
@JvmField val min = 1.min

fun SleepAction(time: Time) = SleepAction(time.s)