package org.firstinspires.ftc.teamcode.evenimente.beclean.robot.systems

import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.acmerobotics.roadrunner.Action
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.Servo
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.evenimente.beclean.robot.CLAW_RAMP_POS
import org.firstinspires.ftc.teamcode.evenimente.beclean.robot.LEFT_CLAW_SERVO_RANGE
import org.firstinspires.ftc.teamcode.evenimente.beclean.robot.LEFT_FINGER_SERVO_RANGE
import org.firstinspires.ftc.teamcode.evenimente.beclean.robot.RIGHT_CLAW_SERVO_RANGE
import org.firstinspires.ftc.teamcode.evenimente.beclean.robot.RIGHT_FINGER_SERVO_RANGE
import org.firstinspires.ftc.teamcode.evenimente.liga_wonder.robot.CONFIG
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

    fun goToAngleNow() {
        realAngle = targetAngle
    }

    val angleBusy get() = targetAngle != realAngle

    fun goToAngle(newAngle: Double) = object : Action {
        var init = true

        override fun run(p: TelemetryPacket): Boolean {
            if (init) {
                init = false
                angle = newAngle
            }

            return angleBusy
        }
    }

    fun openClaw() = object : Action {
        val timer = ElapsedTime()
        var init = true

        override fun run(p: TelemetryPacket): Boolean {
            if (init) {
                init = false
                timer.reset()
                rightFingerPos = 0.0
                leftFingerPos = 0.0
            }

            return timer.seconds() < 1.0
        }
    }

    fun openLeftClaw() = object : Action {
        val timer = ElapsedTime()
        var init = true

        override fun run(p: TelemetryPacket): Boolean {
            if (init) {
                timer.reset()
                init = false
                leftFingerPos = 0.0
            }

            return timer.seconds() < 1.0
        }
    }

    fun closeLeftClaw() = object : Action {
        val timer = ElapsedTime()
        var init = true

        override fun run(p: TelemetryPacket): Boolean {
            if (init) {
                timer.reset()
                init = false
                leftFingerPos = 1.0
            }

            return timer.seconds() < 1.0
        }
    }

    fun openRightClaw() = object : Action {
        val timer = ElapsedTime()
        var init = true

        override fun run(p: TelemetryPacket): Boolean {
            if (init) {
                timer.reset()
                init = false
                rightFingerPos = 0.0
            }

            return timer.seconds() < 1.0
        }
    }

    fun closeClaw() = object : Action {
        val timer = ElapsedTime()
        var init = true

        override fun run(p: TelemetryPacket): Boolean {
            if (init) {
                timer.reset()
                init = false
                rightFingerPos = 1.0
                leftFingerPos = 1.0
            }

            return timer.seconds() < 1.0
        }
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
        val error = targetAngle - realAngle
        realAngle += if (abs(error) < step) {
            error
        } else {
            sign(error) * step
        }
    }
}