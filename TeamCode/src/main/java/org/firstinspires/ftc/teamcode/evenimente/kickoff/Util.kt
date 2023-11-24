package org.firstinspires.ftc.teamcode.evenimente.kickoff

import com.acmerobotics.roadrunner.Vector2d
import kotlin.math.cos
import kotlin.math.sin

fun Vector2d.rotate(angle: Double): Vector2d {
    return Vector2d (
            x = x * cos(angle) - y * sin(angle),
            y = x * sin(angle) + y * cos(angle)
    )
}