package org.firstinspires.ftc.teamcode.robot

import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.acmerobotics.roadrunner.Action
import com.acmerobotics.roadrunner.InstantAction
import com.acmerobotics.roadrunner.SequentialAction
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.Servo
import org.firstinspires.ftc.teamcode.lib.units.SleepAction
import org.firstinspires.ftc.teamcode.lib.units.deg
import org.firstinspires.ftc.teamcode.lib.units.s

@Suppress("unused", "MemberVisibilityCanBePrivate")
class Claw(
    hardwareMap: HardwareMap
) {
    companion object {
        const val fingerOffset = 0.12
        const val clawRampPos = 0.678
        val fingerRange = 0.24..0.55
        val leftFingerRange = 0.0..(1.0 - fingerOffset)
        val rightFingerRange = fingerOffset..1.0
    }
    private val leftAngle = ServoEx(
        servo = hardwareMap.get(Servo::class.java, "leftClaw"),
        maxAngle = 355.deg,
        onPositionUpdate = ::updateRightAngle
    )
    private val rightAngle = ServoEx(
        servo = hardwareMap.get(Servo::class.java, "rightClaw"),
        maxAngle = 355.deg,
        servoDirection = Servo.Direction.REVERSE,
        changeTreshold = 0.0
    )
    private val leftFinger = ServoEx(
        servo = hardwareMap.get(Servo::class.java, "leftFinger"),
        maxAngle = 180.deg,
        positionRange = leftFingerRange
    )
    private val rightFinger = ServoEx(
        servo = hardwareMap.get(Servo::class.java, "leftFinger"),
        maxAngle = 180.deg,
        positionRange = rightFingerRange,
        servoDirection = Servo.Direction.REVERSE
    )

    var clawPosition by leftAngle::position

    var clawTargetPosition by leftAngle::targetPosition

    var clawAngle by leftAngle::angle

    var clawTargetAngle by leftAngle::targetAngle

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

    fun clawToRamp() = clawToPos(clawRampPos)

    fun leftFingerToPos(newPos: Double) = SequentialAction(
        InstantAction { leftFingerPosition = newPos },
        SleepAction(0.1.s)
    )

    fun openLeft() = leftFingerToPos(0.0)

    fun rightFingerToPos(newPos: Double) = SequentialAction(
        InstantAction { rightFingerPosition = newPos },
        SleepAction(0.1.s)
    )

    fun updateRightAngle() {
        rightAngle.position = leftAngle.position
    }
}