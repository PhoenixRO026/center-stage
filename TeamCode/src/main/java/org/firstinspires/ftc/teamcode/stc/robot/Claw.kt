package org.firstinspires.ftc.teamcode.stc.robot

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.Servo

class Claw (hardwareMap: HardwareMap) {
    private val leftFingerServo = hardwareMap.get(Servo::class.java, "lFinger")
    private val rightFingerServo = hardwareMap.get(Servo::class.java, "rFinger")
    private val clawAngleServo = hardwareMap.get(Servo::class.java, "cAngle")

    init {
        leftFingerServo.direction = Servo.Direction.REVERSE
    }

    companion object{
        const val intakeTilt = 0.6746
        const val scoreTilt = 0.7988
        const val closedFinger = 0.6
        const val openFinger = 0.4
    }

    var leftFinger : Double = 0.0
        set(value) {
            val clippedValue = value.coerceIn(0.0, 1.0)
            leftFingerServo.position = clippedValue.ranged(openFinger, closedFinger)
            field = clippedValue
        }

    var rightFinger : Double = 0.0
        set(value) {
            val clippedValue = value.coerceIn(0.0, 1.0)
            rightFingerServo.position = clippedValue.ranged(openFinger, closedFinger)
            field = clippedValue
        }

    var tilt : Double = 0.0
        set(value) {
            val clippedValue = value.coerceIn(0.0, 1.0)
            clawAngleServo.position = clippedValue
            field = clippedValue
        }
}