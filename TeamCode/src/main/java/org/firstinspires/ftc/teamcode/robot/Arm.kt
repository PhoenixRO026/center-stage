package org.firstinspires.ftc.teamcode.robot

import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.Servo
import org.firstinspires.ftc.teamcode.lib.units.Time
import org.firstinspires.ftc.teamcode.lib.units.deg

class Arm(
    hardwareMap: HardwareMap
) {
    private val rightServo = ServoEx(
        hardwareMap.get(Servo::class.java, "rightArm"),
        355.deg,
        onPositionUpdate = ::updateLeftServo
    )
    private val leftServo = ServoEx(
        hardwareMap.get(Servo::class.java, "leftArm"),
        355.deg,
        changeTreshold = 0.0
    )

    private fun updateLeftServo() {
        leftServo.position = rightServo.position
    }

    val position by rightServo::position

    val targetPosition by rightServo::position

    val angle by rightServo::angle

    val targetAngle by rightServo::angle

    fun update(deltaTime: Time) {
        rightServo.update(deltaTime)
    }
}