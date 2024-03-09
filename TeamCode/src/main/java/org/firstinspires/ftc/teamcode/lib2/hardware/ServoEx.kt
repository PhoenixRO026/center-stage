package org.firstinspires.ftc.teamcode.lib2.hardware

interface ServoEx {
    var position: Double

    fun ranged(range: ClosedRange<Double> = 0.0..1.0) = RangedServo(this, range)

    fun speed(speed: Double = SpeedServo.defaultSpeed) = SpeedServo(this, speed)
}