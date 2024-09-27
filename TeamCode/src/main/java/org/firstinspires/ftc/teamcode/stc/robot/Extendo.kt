package org.firstinspires.ftc.teamcode.stc.robot

import com.acmerobotics.roadrunner.InstantAction
import com.acmerobotics.roadrunner.SequentialAction
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.Servo
import com.qualcomm.robotcore.util.Range
import org.firstinspires.ftc.teamcode.lib.units.SleepAction
import org.firstinspires.ftc.teamcode.lib.units.s

class Extendo (hardwareMap: HardwareMap) {
    val leftLiftServo: Servo = hardwareMap.get(Servo::class.java, "lExtend")
    val rightLiftServo: Servo = hardwareMap.get(Servo::class.java, "rExtend")

    companion object{
        const val servoOffset = 0.00
        const val inBot = 0.00
        const val scorePos = 0.50
        val moveWait = 0.8.s
    }

    init {
        leftLiftServo.direction = Servo.Direction.REVERSE

        leftLiftServo.position = inBot.ranged(0.0, 1.0- servoOffset)
        rightLiftServo.position = inBot.ranged(servoOffset, 1.0)
    }

    var pos : Double = inBot
        set(value) {
            val clippedValue = value.coerceIn(0.0, 1.0)
            leftLiftServo.position = clippedValue.ranged(0.0, 1.0 - servoOffset)
            rightLiftServo.position = clippedValue.ranged(servoOffset, 1.0)
            field = clippedValue
        }

    fun goToPos(newPos: Double) = SequentialAction(
            InstantAction { pos = newPos },
            SleepAction(moveWait)
    )

    fun goToScore() = goToPos(scorePos)

    fun goToInBot() = goToPos(inBot)
}