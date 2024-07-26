package org.firstinspires.ftc.teamcode.stc.robot

import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.Servo

class Arm (hardwareMap: HardwareMap) {
    private val servoLeft = hardwareMap.get(Servo::class.java, "leftArmServo")
    private val servoRight = hardwareMap.get(Servo::class.java, "rightArmServo")

    companion object{
        const val servoOffset = 0.0
    }
    init {
        servoLeft.direction = Servo.Direction.REVERSE

        servoLeft.scaleRange(servoOffset, 1.0)
        servoRight.scaleRange(0.0, 1.0 - servoOffset)
    }

    var pos : Double
        get() = servoLeft.position
        set(value) {
            servoLeft.position = value
            servoRight.position = value
        }

}