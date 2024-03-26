@file:Suppress("unused")

package com.phoenix.phoenixlib.units

import com.acmerobotics.roadrunner.Rotation2d
import kotlin.math.PI

data class Angle(@JvmField var deg: Double) {
    val rev get() = deg.degToRev()
    val rad get() = deg.degToRad()

    val rotation2d get() = Rotation2d.exp(rad)

    operator fun plus(other: Angle) = Angle(deg + other.deg)
    operator fun minus(other: Angle) = Angle(deg - other.deg)
    operator fun times(scalar: Number) = Angle(deg * scalar.toDouble())
    operator fun div(scalar: Number) = Angle(deg / scalar.toDouble())
    operator fun div(other: Angle) = deg / other.deg
    operator fun div(time: Time) = AngularVelocity(this, time)
    operator fun unaryMinus() = Angle(-deg)

    override fun toString() = "$deg " + if (deg == 1.0) "degree" else "degrees"

    fun coerceIn(minAng: Angle, maxAng: Angle) = Angle(deg.coerceIn(minAng.deg, maxAng.deg))
}

fun Number.degToRev() = toDouble() / 360.0
fun Number.degToRad() = toDouble() / 180.0 * PI
fun Number.revToDeg() = toDouble() * 360.0
fun Number.revToRad() = toDouble() * 2 * PI
fun Number.radToDeg() = toDouble() * 180 / PI
fun Number.radToRev() = toDouble() / (2 * PI)

val Number.rad get() = Angle(radToDeg())
val Number.deg get() = Angle(toDouble())
val Number.rev get() = Angle(revToDeg())

fun rad(number: Double) = number.rad
fun deg(number: Double) = number.deg
fun rev(number: Double) = number.rev

@JvmField val rad = 1.rad
@JvmField val deg = 1.deg
@JvmField val rev = 1.rev

val Rotation2d.angle get() = Angle(toDouble() / PI * 180.0)

fun sin(angle: Angle) = kotlin.math.sin(angle.rad)
fun cos(angle: Angle) = kotlin.math.cos(angle.rad)