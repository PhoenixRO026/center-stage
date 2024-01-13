@file:Suppress("unused")

package com.phoenix_ro026.phoenixlib.old_units

import com.acmerobotics.roadrunner.Rotation2d

class AngularVelocity(val rotation: Rotation, val duration: Duration) {
    override fun toString() = "$rotation per $duration"
    fun toUnit(angularVelocity: AngularVelocity): Double {
        val rotationVal = rotation.toUnit(angularVelocity.rotation)
        val durationVal = duration.toUnit(angularVelocity.duration)
        return rotationVal / durationVal
    }
    fun toRotation2d() = Rotation2d.exp(toUnit(radian.per(second)))
    fun asUnit(angularVelocity: AngularVelocity) = AngularVelocity(rotationAsUnit(angularVelocity), angularVelocity.duration)
    fun rotationAsUnit(unit: AngularVelocity): Rotation {
        val durationRatio = duration.toUnit(unit.duration) / unit.duration.value //2 / 1 = 2         1 meter per 2 seconds = 0.5 meter per 1 second
        return rotation.asUnit(unit.rotation) / durationRatio
    }
    operator fun plus(d: AngularVelocity) = AngularVelocity(rotation + d.rotationAsUnit(this), duration)
    operator fun minus(d: AngularVelocity) = AngularVelocity(rotation - d.rotationAsUnit(this), duration)
    operator fun unaryMinus() = AngularVelocity(-rotation, duration)
    operator fun times(z: Double) = AngularVelocity(rotation * z, duration)
    operator fun div(z: Double) = AngularVelocity(rotation / z, duration)
}

val Number.rpm get() = rev.per(minute)
val Number.radSec get() = rad.per(second)
val Number.degSec get() = deg.per(second)

val rotPerMin = 1.rpm
val radPerSec = 1.radSec
val degPerSec = 1.degSec

fun Rotation.per(duration: Duration) = AngularVelocity(this, duration)