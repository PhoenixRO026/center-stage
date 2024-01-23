package org.firstinspires.ftc.teamcode.lib.units

import com.acmerobotics.roadrunner.Rotation2d

class Angle(val rad: Double) {
    val deg get() = rad * 180.0 / Math.PI
    val rev get() = rad / 2.0 / Math.PI

    val rotation2d get() = Rotation2d.exp(rad)

    operator fun plus(other: Angle) = Angle(rad + other.rad)
    operator fun minus(other: Angle) = Angle(rad - other.rad)
    operator fun times(scalar: Number) = Angle(rad * scalar.toDouble())
    operator fun div(scalar: Number) = Angle(rad / scalar.toDouble())
    operator fun div(other: Angle) = rad / other.rad
    operator fun div(time: Time) = AngularVelocity(this, time)
    operator fun unaryMinus() = Angle(-rad)

    override fun toString() = "$rad " + if (rad == 1.0) "radian" else "radians"
}

val Number.rad get() = Angle(toDouble())
val Number.deg get() = Angle(toDouble() / 180.0 * Math.PI)
val Number.rev get() = Angle(toDouble() * 2.0 * Math.PI)

fun rad(number: Double) = number.rad
fun deg(number: Double) = number.deg
fun rev(number: Double) = number.rev

@JvmField val rad = 1.rad
@JvmField val deg = 1.deg
@JvmField val rev = 1.rev

val Rotation2d.angle get() = Angle(log())