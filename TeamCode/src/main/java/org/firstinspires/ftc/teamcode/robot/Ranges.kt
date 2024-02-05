package org.firstinspires.ftc.teamcode.robot

fun Double.scaleTo(range: ClosedRange<Double>): Double {
    return range.start + this * (range.endInclusive - range.start)
}

fun Double.reverseScale(range: ClosedRange<Double>): Double {
    return (this - range.start) / (range.endInclusive - range.start)
}