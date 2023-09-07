package org.firstinspires.ftc.teamcode

import com.acmerobotics.roadrunner.Vector2d
import kotlin.math.cos
import kotlin.math.sin

fun Vector2d.rotate(double: Double): Vector2d {
    return Vector2d (
            x = x * cos(double) - y * sin(double),
            y = x * sin(double) + y * cos(double)
    )
}