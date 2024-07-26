package org.firstinspires.ftc.teamcode.stc.robot

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.Servo

class Claw (hardwareMap: HardwareMap) {
    private val leftFingerServo = hardwareMap.get(Servo::class.java, "lFinger")
    private val rightFingerServo = hardwareMap.get(Servo::class.java, "rFinger")
    private val clawAngleServo = hardwareMap.get(Servo::class.java, "cAngle")

    var leftFinger : Number
        get() = leftFingerServo.position
        set(value) {
            leftFingerServo.position = value.toDouble()
        }

    var rightFinger : Number
        get() = rightFingerServo.position
        set(value) {
            rightFingerServo.position = value.toDouble()
        }

    var tilt : Double
        get() = clawAngleServo.position
        set(value) {
            clawAngleServo.position = value
        }
}