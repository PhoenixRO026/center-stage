package org.firstinspires.ftc.teamcode.robot

import com.acmerobotics.roadrunner.InstantAction
import com.acmerobotics.roadrunner.ParallelAction
import com.acmerobotics.roadrunner.SequentialAction
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.Servo
import org.firstinspires.ftc.teamcode.lib.units.SleepAction
import org.firstinspires.ftc.teamcode.lib.units.Time
import org.firstinspires.ftc.teamcode.lib.units.deg
import org.firstinspires.ftc.teamcode.lib.units.s
import org.firstinspires.ftc.teamcode.robot.hardware.ServoEx
import org.firstinspires.ftc.teamcode.robot.hardware.reverseScale
import org.firstinspires.ftc.teamcode.robot.hardware.scaleTo

@Suppress("unused", "MemberVisibilityCanBePrivate")
class Claw(
    hardwareMap: HardwareMap
) {
    companion object {
        const val fingerOffset = 0.12
        const val rampPos = 0.653
        const val scorePos = 0.819
        const val fingerRampPos = 0.193
        val fingerRange = 0.24..0.55
        val leftFingerRange = 0.0..(1.0 - fingerOffset)
        val rightFingerRange = fingerOffset..1.0

        fun clawH(hardwareMap: HardwareMap) = Claw(
            hardwareMap = hardwareMap
        )

        fun HardwareMap.claw() = Claw(
            hardwareMap = this
        )
    }
    private val angleServo = ServoEx(
        servo = hardwareMap.get(Servo::class.java, "clawAngle"),
        maxAngle = 355.deg,
        changeTreshold = 0.0,
        speed = 0.5
        //onPositionUpdate = ::updateRightAngle
    )
    /*private val rightAngle = ServoEx(
        servo = hardwareMap.get(Servo::class.java, "rightClaw"),
        maxAngle = 355.deg,
        servoDirection = Servo.Direction.REVERSE,
        changeTreshold = 0.0
    )*/
    private val leftFinger = ServoEx(
        servo = hardwareMap.get(Servo::class.java, "leftFinger"),
        maxAngle = 180.deg,
        positionRange = leftFingerRange
    )
    private val rightFinger = ServoEx(
        servo = hardwareMap.get(Servo::class.java, "rightFinger"),
        maxAngle = 180.deg,
        positionRange = rightFingerRange,
        servoDirection = Servo.Direction.REVERSE
    )

    var clawPosition by angleServo::position

    var clawTargetPosition by angleServo::targetPosition

    var clawAngle by angleServo::angle

    var clawTargetAngle by angleServo::targetAngle

    var leftFingerPosition
        get() = leftFinger.position.reverseScale(fingerRange)
        set(value) {
            leftFinger.position = value.scaleTo(fingerRange)
        }

    var leftFingerTargetPosition
        get() = leftFinger.targetPosition.reverseScale(fingerRange)
        set(value) {
            leftFinger.targetPosition = value.scaleTo(fingerRange)
        }

    var leftFingerAngle by leftFinger::angle

    var leftFingerTargetAngle by leftFinger::targetAngle

    var rightFingerPosition
        get() = rightFinger.position.reverseScale(fingerRange)
        set(value) {
            rightFinger.position = value.scaleTo(fingerRange)
        }

    var rightFingerTargetPosition
        get() = rightFinger.targetPosition.reverseScale(fingerRange)
        set(value) {
            rightFinger.targetPosition = value.scaleTo(fingerRange)
        }

    var rightFingerAngle by rightFinger::angle

    var rightFingerTargetAngle by rightFinger::targetAngle

    fun clawToPos(newPos: Double) = SequentialAction(
        InstantAction { clawPosition = newPos },
        SleepAction(0.1.s)
    )

    fun clawToRamp() = clawToPos(rampPos)

    fun leftFingerToPos(newPos: Double) = SequentialAction(
        InstantAction { leftFingerPosition = newPos },
        SleepAction(0.1.s)
    )

    fun openLeft() = leftFingerToPos(0.0)

    fun closeLeft() = leftFingerToPos(1.0)

    fun rightFingerToPos(newPos: Double) = SequentialAction(
        InstantAction { rightFingerPosition = newPos },
        SleepAction(0.1.s)
    )

    fun openRight() = rightFingerToPos(0.0)

    fun closeRight() = rightFingerToPos(1.0)

    fun closeClaw() = ParallelAction(
        openRight(),
        openLeft()
    )

    fun update(deltaTime: Time) {
        angleServo.update(deltaTime)
        leftFinger.update(deltaTime)
        rightFinger.update(deltaTime)
    }

    init {
        clawPosition = rampPos
        leftFingerPosition = 1.0
        rightFingerPosition = 1.0
    }
    /*fun updateRightAngle() {
        rightAngle.position = angleServo.position
    }*/
}