@file:Suppress("unused")

package com.phoenix_ro026.phoenixlib.units

import com.acmerobotics.roadrunner.Pose2d
import com.acmerobotics.roadrunner.Vector2d

class Velocity(val distance: Distance, val duration: Duration) {
    override fun toString() = "$distance per $duration"
    fun toUnit(velocity: Velocity): Double {
        val distanceVal = distance.toUnit(velocity.distance)
        val durationVal = duration.toUnit(velocity.duration)
        return distanceVal / durationVal
    }
    fun asUnit(velocity: Velocity) = Velocity(distanceAsUnit(velocity), velocity.duration)
    fun distanceAsUnit(unit: Velocity): Distance {
        //2 / 1 = 2    1 meter per 2 seconds = 0.5 meter per 1 second
        val durationRatio = duration.toUnit(unit.duration) / unit.duration.value
        return distance.asUnit(unit.distance) / durationRatio
    }
    operator fun plus(d: Velocity) = Velocity(distance + d.distanceAsUnit(this), duration)
    operator fun minus(d: Velocity) = Velocity(distance - d.distanceAsUnit(this), duration)
    operator fun unaryMinus() = Velocity(-distance, duration)
    operator fun times(z: Double) = Velocity(distance * z, duration)
    operator fun div(z: Double) = Velocity(distance / z, duration)
}

val Number.cmSec get() = cm.per(second)
val Number.inchSec get() = inch.per(second)
val Number.meterSec get() = meters.per(second)

val cmPerSec = 1.cmSec
val inchPerSec = 1.inchSec
val meterPerSec = 1.meterSec

fun Distance.per(duration: Duration): Velocity = Velocity(this, duration)

data class Velocity2d(@JvmField val x: Velocity, @JvmField val y: Velocity) {
    fun toVector2d() = Vector2d(x.toUnit(inch.per(second)), y.toUnit(inch.per(second)))
    operator fun plus(v: Velocity2d) = Velocity2d(x + v.x, y + v.y)
    operator fun minus(v: Velocity2d) = Velocity2d(x - v.x, y - v.y)
    operator fun unaryMinus() = Velocity2d(-x, -y)
    operator fun times(z: Double) = Velocity2d(x * z, y * z)
    operator fun div(z: Double) = Velocity2d(x / z, y / z)
}

data class PoseVelocity(@JvmField val position: Velocity2d, @JvmField val heading: AngularVelocity) {
    constructor(x: Velocity, y: Velocity, heading: AngularVelocity) : this(Velocity2d(x, y), heading)
    fun toPose2d() = Pose2d(position.toVector2d(), heading.toRotation2d())
    operator fun plus(p: PoseVelocity) = PoseVelocity(position + p.position, heading + p.heading)
    operator fun minus(p: PoseVelocity) = PoseVelocity(position - p.position, heading - p.heading)
    operator fun plus(v: Velocity2d) = PoseVelocity(position + v, heading)
    operator fun minus(v: Velocity2d) = PoseVelocity(position - v, heading)
    operator fun plus(a: AngularVelocity) = PoseVelocity(position, heading + a)
    operator fun minus(a: AngularVelocity) = PoseVelocity(position, heading - a)
    operator fun unaryMinus() = PoseVelocity(-position, -heading)
    operator fun times(z: Double) = PoseVelocity(position * z, heading * z)
    operator fun div(z: Double) = PoseVelocity(position / z, heading / z)
}