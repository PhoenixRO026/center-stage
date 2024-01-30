package org.firstinspires.ftc.teamcode.lib.units

import com.acmerobotics.roadrunner.Pose2d
import com.acmerobotics.roadrunner.Vector2d
import kotlin.math.cos
import kotlin.math.sin

class Distance(val m: Double) {
    val mm get() = m * 1000.0
    val cm get() = m * 100.0
    val inch get() = m / 0.0254

    val x get() = Distance2d(this, 0.m)
    val y get() = Distance2d(0.m, this)

    operator fun plus(other: Distance) = Distance(m + other.m)
    operator fun minus(other: Distance) = Distance(m - other.m)
    operator fun times(scalar: Number) = Distance(m * scalar.toDouble())
    operator fun div(scalar: Number) = Distance(m / scalar.toDouble())
    operator fun div(other: Distance) = m / other.m
    operator fun div(time: Time) = Velocity(this, time)
    operator fun unaryMinus() = Distance(-m)

    override fun toString() = "$m " + if (m == 1.0) "meter" else "meters"
}

val Number.m get() = Distance(toDouble())
val Number.mm get() = Distance(toDouble() / 1000.0)
val Number.cm get() = Distance(toDouble() / 100.0)
val Number.inch get() = Distance(toDouble() * 0.0254)

fun m(number: Double) = number.m
fun mm(number: Double) = number.mm
fun cm(number: Double) = number.cm
fun inch(number: Double) = number.inch

@JvmField val m = 1.m
@JvmField val mm = 1.mm
@JvmField val cm = 1.cm
@JvmField val inch = 1.inch

fun Vector2d(x: Distance, y: Distance) = Distance2d(x, y)

data class Distance2d(val x: Distance, val y: Distance) {
    operator fun plus(other: Distance2d) = Distance2d(x + other.x, y + other.y)
    operator fun minus(other: Distance2d) = Distance2d(x - other.x, y - other.y)
    operator fun times(scalar: Number) = Distance2d(x * scalar, y * scalar)
    operator fun div(scalar: Number) = Distance2d(x / scalar, y / scalar)
    operator fun unaryMinus() = Distance2d(-x, -y)

    val meters get() = Vector2d(x.m, y.m)
    val mm get() = Vector2d(x.mm, y.mm)
    val cm get() = Vector2d(x.cm, y.cm)
    val inch get() = Vector2d(x.inch, y.inch)

    val vector2d get() = inch
}

val Vector2d.m get() = Distance2d(x.m, y.m)
val Vector2d.mm get() = Distance2d(x.mm, y.mm)
val Vector2d.cm get() = Distance2d(x.cm, y.cm)
val Vector2d.inch get() = Distance2d(x.inch, y.inch)
val Vector2d.dist get() = inch

fun m(vector: Vector2d) = vector.m
fun mm(vector: Vector2d) = vector.mm
fun cm(vector: Vector2d) = vector.cm
fun inch(vector: Vector2d) = vector.inch
fun dist(vector: Vector2d) = vector.dist

data class Pose(val position: Distance2d, val heading: Angle) {

    constructor(x: Distance, y: Distance, heading: Angle) : this(Distance2d(x, y), heading)

    operator fun plus(other: Pose) = Pose(position + other.position, heading + other.heading)
    operator fun plus(other: Distance2d) = Pose(position + other, heading)
    operator fun plus(other: Angle) = Pose(position, heading + other)
    operator fun minus(other: Pose) = Pose(position - other.position, heading - other.heading)
    operator fun minus(other: Distance2d) = Pose(position - other, heading)
    operator fun minus(other: Angle) = Pose(position, heading - other)

    val pose2d get() = Pose2d(position.inch, heading.rad)
}

val Pose2d.m get() = Pose(position.m, heading.angle)
val Pose2d.mm get() = Pose(position.mm, heading.angle)
val Pose2d.cm get() = Pose(position.cm, heading.angle)
val Pose2d.inch get() = Pose(position.inch, heading.angle)
val Pose2d.pose get() = inch

fun m(pose2d: Pose2d) = pose2d.m
fun mm(pose2d: Pose2d) = pose2d.mm
fun cm(pose2d: Pose2d) = pose2d.cm
fun inch(pose2d: Pose2d) = pose2d.inch
fun pose(pose2d: Pose2d) = pose2d.pose

fun Vector2d.rotate(radians: Double) = Vector2d(
    x * cos(radians) - y * sin(radians),
    x * sin(radians) + y * cos(radians)
)