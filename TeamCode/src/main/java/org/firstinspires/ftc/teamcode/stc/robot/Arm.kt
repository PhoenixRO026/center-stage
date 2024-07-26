package org.firstinspires.ftc.teamcode.stc.robot

import com.acmerobotics.roadrunner.InstantAction
import com.acmerobotics.roadrunner.SequentialAction
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.Servo
import com.qualcomm.robotcore.util.Range
import org.firstinspires.ftc.teamcode.lib.units.SleepAction
import org.firstinspires.ftc.teamcode.lib.units.s

class Arm (hardwareMap: HardwareMap) {
    val servoLeft = hardwareMap.get(Servo::class.java, "leftArmServo")
    val servoRight = hardwareMap.get(Servo::class.java, "rightArmServo")

    companion object{
        const val servoOffset = 0.01
        const val intakePos = 0.5512
        const val scorePos = 0.8669
        val moveWait = 0.8.s
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

    fun goToPos(newPos: Double) = SequentialAction(
        InstantAction { pos = newPos },
        SleepAction(moveWait)
    )

    fun goToScore() = goToPos(scorePos)

    fun goToIntake() = goToPos(intakePos)
}