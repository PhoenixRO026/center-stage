package org.firstinspires.ftc.teamcode.stc.robot

import com.acmerobotics.roadrunner.InstantAction
import com.acmerobotics.roadrunner.SequentialAction
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.Servo
import org.firstinspires.ftc.teamcode.lib.units.SleepAction
import org.firstinspires.ftc.teamcode.lib.units.s

class Claw (hardwareMap: HardwareMap) {
    private val leftFingerServo = hardwareMap.get(Servo::class.java, "lFinger")
    private val rightFingerServo = hardwareMap.get(Servo::class.java, "rFinger")
    private val clawAngleServo = hardwareMap.get(Servo::class.java, "cAngle")
    private val clawRotationServo = hardwareMap.get(Servo::class.java, "cRotaton")

    init {
        leftFingerServo.direction = Servo.Direction.REVERSE

        leftFingerServo.position = closedFinger
        rightFingerServo.position = closedFinger
        clawAngleServo.position = intakeTilt
    }

    companion object{
        const val intakeTilt = 0.6746
        const val scoreTilt = 0.7988
        const val closedFinger = 0.6
        const val openFinger = 0.4
        val tiltWait = 0.8.s
        val fingerWait = 0.5.s
    }

    var leftFinger : Double = 1.0
        set(value) {
            val clippedValue = value.coerceIn(0.0, 1.0)
            leftFingerServo.position = clippedValue.ranged(openFinger, closedFinger)
            field = clippedValue
        }

    var rightFinger : Double = 1.0
        set(value) {
            val clippedValue = value.coerceIn(0.0, 1.0)
            rightFingerServo.position = clippedValue.ranged(openFinger, closedFinger)
            field = clippedValue
        }

    var tilt : Double = intakeTilt
        set(value) {
            val clippedValue = value.coerceIn(0.0, 1.0)
            clawAngleServo.position = clippedValue
            field = clippedValue
        }



    fun tiltToPos(newTilt: Double) = SequentialAction(
        InstantAction { tilt = newTilt },
        SleepAction(tiltWait)
    )

    fun tiltToScore() = tiltToPos(scoreTilt)

    fun tiltToIntake() = tiltToPos(intakeTilt)

    fun openLeft() = SequentialAction(
        InstantAction { leftFinger = openFinger },
        SleepAction(fingerWait)
    )

    fun openRight() = SequentialAction(
        InstantAction { rightFinger = openFinger },
        SleepAction(fingerWait)
    )

    fun closeLeft() = SequentialAction(
        InstantAction { leftFinger = closedFinger },
        SleepAction(fingerWait)
    )

    fun closeRight() = SequentialAction(
        InstantAction { rightFinger = closedFinger },
        SleepAction(fingerWait)
    )

    fun openClaw() = SequentialAction(
        InstantAction {
            leftFinger = openFinger
            rightFinger = openFinger
                      },
        SleepAction(fingerWait)
    )

    fun closeClaw() = SequentialAction(
        InstantAction {
            leftFinger = closedFinger
            rightFinger = closedFinger
                      },
        SleepAction(fingerWait)
    )
}