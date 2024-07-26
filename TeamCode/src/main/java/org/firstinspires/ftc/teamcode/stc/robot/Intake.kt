package org.firstinspires.ftc.teamcode.stc.robot

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.Servo

class Intake (hardwareMap: HardwareMap) {
    val motor = hardwareMap.get(DcMotor::class.java, "intake")
    val servo = hardwareMap.get(Servo::class.java, "intakeServo")

    var pos : Number
        get() = servo.position
        set(value) {
            servo.position = value.toDouble()
        }

    var power : Number
        get() = motor.power
        set(value) {
            motor.power = value.toDouble()
        }
}