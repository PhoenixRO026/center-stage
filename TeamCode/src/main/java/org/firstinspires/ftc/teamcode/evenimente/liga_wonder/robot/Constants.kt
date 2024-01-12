package org.firstinspires.ftc.teamcode.evenimente.liga_wonder.robot

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot
import com.qualcomm.robotcore.hardware.DcMotorSimple

///ROBOT FRONT IS INTAKE

const val ARM_SERVO_OFFSET = 0.015
const val FINGERS_SERVO_OFFSET = 0.17

const val FINGERS_OPEN_LIMIT = 0.15
const val FINGERS_INTAKE_OPEN_POS = 0.3
const val FINGERS_CLOSE_LIMIT = 0.61

const val HANG_POS = 600

const val ARM_SCORE_POS = 0.46
const val CLAW_SCORE_POS = 0.915

val FINGERS_RANGE = FINGERS_OPEN_LIMIT..FINGERS_CLOSE_LIMIT

const val ARM_RAMP_POS = 0.4
const val CLAW_RAMP_POS = 0.8

val LEFT_ARM_SERVO_RANGE = ARM_SERVO_OFFSET..1.0
val RIGHT_ARM_SERVO_RANGE = 0.0..(1 - ARM_SERVO_OFFSET)

val LEFT_CLAW_SERVO_RANGE = 0.0..1.0
val RIGHT_CLAW_SERVO_RANGE = 0.0..1.0

val LEFT_FINGER_SERVO_RANGE = 0.0..(1.0 - FINGERS_SERVO_OFFSET)
val RIGHT_FINGER_SERVO_RANGE = FINGERS_SERVO_OFFSET..1.0

val LEFT_BACK_DRIVE_DIRECTION = DcMotorSimple.Direction.FORWARD
val LEFT_FRONT_DRIVE_DIRECTION = DcMotorSimple.Direction.FORWARD
val RIGHT_BACK_DRIVE_DIRECTION = DcMotorSimple.Direction.REVERSE
val RIGHT_FRONT_DRIVE_DIRECTION = DcMotorSimple.Direction.REVERSE

val IMU_LOGO_FACING_DIRECTION = RevHubOrientationOnRobot.LogoFacingDirection.LEFT
val IMU_USB_FACING_DIRECTION = RevHubOrientationOnRobot.UsbFacingDirection.UP

const val PLANE_SERVO_CLOSED = 0.64
const val PLANE_SERVO_OPENED = 0.5

val PLANE_RANGE = PLANE_SERVO_OPENED..PLANE_SERVO_CLOSED

data object CONFIG {
    const val LEFT_ARM = "leftArm"
    const val RIGHT_ARM = "rightArm"
    const val CAMERA = "Webcam 1"
    const val LEFT_LIFT = "leftLift"
    const val RIGHT_LIFT = "rightLift"
    const val INTAKE = "intake"

    const val LEFT_BACK_DRIVE = "leftBack"
    const val LEFT_FRONT_DRIVE = "leftFront"
    const val RIGHT_BACK_DRIVE = "rightBack"
    const val RIGHT_FRONT_DRIVE = "rightFront"

    const val LEFT_CLAW = "leftClaw"
    const val RIGHT_CLAW = "rightClaw"

    const val LEFT_FINGER = "leftFinger"
    const val RIGHT_FINGER = "rightFinger"

    const val PLANE = "plane"
}