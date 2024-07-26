package org.firstinspires.ftc.teamcode.stc.robot

import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.Servo
import com.qualcomm.robotcore.util.Range

class Arm (hardwareMap: HardwareMap) {
    val servoLeft = hardwareMap.get(Servo::class.java, "leftArmServo")
    val servoRight = hardwareMap.get(Servo::class.java, "rightArmServo")

    companion object{
        const val servoOffset = 0.01
        const val intakePos = 0.5512
        const val scorePos = 0.8669
    }
    init {
        servoLeft.direction = Servo.Direction.REVERSE
    }

    var pos : Double = 0.0
        set(value) {
            val clippedValue = value.coerceIn(0.0, 1.0)
            servoLeft.position = clippedValue.ranged(0.0, 1.0 - servoOffset)
            servoRight.position = clippedValue.ranged(servoOffset, 1.0)
            field = clippedValue
        }
}