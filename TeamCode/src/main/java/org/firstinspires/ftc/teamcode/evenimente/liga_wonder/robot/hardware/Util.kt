package org.firstinspires.ftc.teamcode.evenimente.liga_wonder.robot.hardware

fun Double.scaleTo(range: ClosedRange<Double>): Double {
    return range.start + this * (range.endInclusive - range.start)
}