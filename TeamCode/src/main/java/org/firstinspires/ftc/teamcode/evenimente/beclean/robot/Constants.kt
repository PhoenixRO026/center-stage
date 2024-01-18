package org.firstinspires.ftc.teamcode.evenimente.beclean.robot

val INTAKE_RANGE = 0.489..0.783

const val CLAW_PIXEL_DROP_ANGLE = 0.321

const val CLAW_STACK_ANGLE = 0.2

const val ARM_PIXEL_DROP_POSE = 0.438

const val ARM_STACK_POSE = 0.44

const val LIFT_PASS_POSE = 1400

const val LIFT_PIXEL_DROP_POSE = 204

const val LIFT_STACK_POSE = 500

const val ARM_SERVO_OFFSET = 0.015
const val FINGERS_SERVO_OFFSET = 0.17

const val FINGERS_OPEN_LIMIT = 0.15
const val FINGERS_INTAKE_OPEN_POS = 0.3
const val FINGERS_CLOSE_LIMIT = 0.61

const val HANG_POS = 600

const val ARM_SCORE_POS = 0.46
const val CLAW_SCORE_ANGLE = 0.825

val FINGERS_RANGE = FINGERS_OPEN_LIMIT..FINGERS_CLOSE_LIMIT

const val ARM_RAMP_POS = 0.3873
const val CLAW_RAMP_ANGLE = 0.702

val LEFT_ARM_SERVO_RANGE = ARM_SERVO_OFFSET..1.0
val RIGHT_ARM_SERVO_RANGE = 0.0..(1 - ARM_SERVO_OFFSET)

val LEFT_CLAW_SERVO_RANGE = 0.0..1.0
val RIGHT_CLAW_SERVO_RANGE = 0.0..1.0

val CLAW_RANGE = 0.175..0.89

val LEFT_FINGER_SERVO_RANGE = 0.0..(1.0 - FINGERS_SERVO_OFFSET)
val RIGHT_FINGER_SERVO_RANGE = FINGERS_SERVO_OFFSET..1.0


const val PLANE_SERVO_CLOSED = 0.94
const val PLANE_SERVO_OPENED = 0.67

val PLANE_RANGE = PLANE_SERVO_OPENED..PLANE_SERVO_CLOSED