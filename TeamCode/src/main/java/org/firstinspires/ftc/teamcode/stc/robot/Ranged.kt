package org.firstinspires.ftc.teamcode.stc.robot

import com.qualcomm.robotcore.util.Range

fun Double.ranged(min: Double, max: Double) = Range.scale(this, 0.0, 1.0, min, max)