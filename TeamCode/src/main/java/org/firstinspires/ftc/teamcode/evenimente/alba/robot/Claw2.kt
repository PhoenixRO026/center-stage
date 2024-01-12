package org.firstinspires.ftc.teamcode.evenimente.alba.robot

import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.Servo
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.evenimente.liga_wonder.robot.ARM_RAMP_POS
import org.firstinspires.ftc.teamcode.evenimente.liga_wonder.robot.CLAW_RAMP_POS
import org.firstinspires.ftc.teamcode.evenimente.liga_wonder.robot.CONFIG
import org.firstinspires.ftc.teamcode.evenimente.liga_wonder.robot.LEFT_CLAW_SERVO_RANGE
import org.firstinspires.ftc.teamcode.evenimente.liga_wonder.robot.LEFT_FINGER_SERVO_RANGE
import org.firstinspires.ftc.teamcode.evenimente.liga_wonder.robot.RIGHT_CLAW_SERVO_RANGE
import org.firstinspires.ftc.teamcode.evenimente.liga_wonder.robot.RIGHT_FINGER_SERVO_RANGE
import org.firstinspires.ftc.teamcode.evenimente.liga_wonder.robot.hardware.ServoEx
import org.firstinspires.ftc.teamcode.evenimente.liga_wonder.robot.hardware.scaleTo
import kotlin.math.abs
import kotlin.math.sign

class Claw2(
    hardwareMap: HardwareMap,
    private val telemetry: Telemetry? = null,
    private val fingerRange: ClosedRange<Double> = 0.0..1.0
) {
    private val leftAngle = ServoEx(hardwareMap, CONFIG.LEFT_CLAW, LEFT_CLAW_SERVO_RANGE, telemetry, Servo.Direction.REVERSE)
    private val rightAngle = ServoEx(hardwareMap, CONFIG.RIGHT_CLAW, RIGHT_CLAW_SERVO_RANGE, telemetry)
    private val leftFinger = ServoEx(hardwareMap, CONFIG.LEFT_FINGER, LEFT_FINGER_SERVO_RANGE, telemetry)
    private val rightFinger = ServoEx(hardwareMap, CONFIG.RIGHT_FINGER, RIGHT_FINGER_SERVO_RANGE, telemetry, Servo.Direction.REVERSE)

    private var realAngle = 0.0
        set(value) {
            if (value != field) {
                leftAngle.position = value
                rightAngle.position = value
            }
            field = value
        }

    private var targetAngle = 0.0
    private val speed = 0.01

    var angle: Double = CLAW_RAMP_POS
        set(value) {
            targetAngle = value
            field = value
        }

    var leftFingerPos: Double = 0.0
        set(value) {
            val scaledValue = value.coerceIn(0.0, 1.0).scaleTo(fingerRange)
            leftFinger.position = scaledValue
            field = value.coerceIn(0.0..1.0)
        }

    var rightFingerPos: Double = 0.0
        set(value) {
            val scaledValue = value.coerceIn(0.0, 1.0).scaleTo(fingerRange)
            rightFinger.position = scaledValue
            field = value.coerceIn(0.0..1.0)
        }

    init {
        angle = CLAW_RAMP_POS
        realAngle = targetAngle
        leftFingerPos = 1.0
        rightFingerPos = 1.0
    }

    fun update(deltaTime: Double) {
        val step = speed * deltaTime
        realAngle += if (abs(targetAngle - realAngle) < step) {
            targetAngle - realAngle
        } else {
            sign(targetAngle - realAngle) * step
        }
    }
}