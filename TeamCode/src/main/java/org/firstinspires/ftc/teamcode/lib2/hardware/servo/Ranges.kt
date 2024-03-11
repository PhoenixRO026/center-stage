package org.firstinspires.ftc.teamcode.lib2.hardware.servo

import com.qualcomm.robotcore.util.Range

fun Double.scaleTo(range: ClosedRange<Double>): Double {
    return Range.scale(this, 0.0, 1.0, range.start, range.endInclusive)
}

fun Double.reverseScale(range: ClosedRange<Double>): Double {
    return Range.scale(this, range.start, range.endInclusive, 0.0, 1.0)
}