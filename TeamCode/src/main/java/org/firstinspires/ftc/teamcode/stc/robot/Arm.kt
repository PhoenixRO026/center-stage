package org.firstinspires.ftc.teamcode.stc.robot

import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.Servo
import com.qualcomm.robotcore.util.Range

class Arm (hardwareMap: HardwareMap) {
    private val servoLeft = hardwareMap.get(Servo::class.java, "leftArmServo")
    private val servoRight = hardwareMap.get(Servo::class.java, "rightArmServo")

    companion object{
        const val servoOffset = 0.01
    }
    init {
        servoLeft.direction = Servo.Direction.REVERSE
    }

    var pos : Double = 0.0
        set(value) {
            servoLeft.position = Range.scale(value, 0.0, 1.0, 0.0, 1.0 - servoOffset)
            servoRight.position = Range.scale(value, 0.0, 1.0, servoOffset, 1.0)
            field = value.coerceIn(0.0, 1.0)
        }

}