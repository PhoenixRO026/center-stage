package org.firstinspires.ftc.teamcode.evenimente.beclean.robot.systems

import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.acmerobotics.roadrunner.Action
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.evenimente.beclean.robot.LEFT_ARM_SERVO_RANGE
import org.firstinspires.ftc.teamcode.evenimente.beclean.robot.RIGHT_ARM_SERVO_RANGE
import org.firstinspires.ftc.teamcode.evenimente.liga_wonder.robot.CONFIG
import org.firstinspires.ftc.teamcode.evenimente.liga_wonder.robot.hardware.ServoEx
import org.firstinspires.ftc.teamcode.evenimente.liga_wonder.robot.hardware.scaleTo
import kotlin.math.abs
import kotlin.math.sign

class Arm2(
    hardwareMap: HardwareMap,
    private val telemetry: Telemetry? = null,
    val range: ClosedRange<Double> = 0.0..1.0
) {
    private val rightServo = ServoEx(hardwareMap, CONFIG.RIGHT_ARM, RIGHT_ARM_SERVO_RANGE, telemetry)
    private val leftServo = ServoEx(hardwareMap, CONFIG.LEFT_ARM, LEFT_ARM_SERVO_RANGE, telemetry)

    private var realPosition = 0.0
        set(value) {
            if (value != field) {
                leftServo.position = value
                rightServo.position = value
            }
            field = value
        }
    private var targetPosition = 0.0
    private val speed = 0.01

    val isBusy get() = targetPosition != realPosition

    fun goToPos(pos: Double) = object : Action {
        var init = true

        override fun run(p: TelemetryPacket): Boolean {
            if (init) {
                init = false
                position = pos
            }

            return isBusy
        }
    }

    var position: Double = 0.0
        set(value) {
            val scaledValue = value.coerceIn(0.0, 1.0).scaleTo(range)
            targetPosition = scaledValue
            //rightServo.position = scaledValue
            //leftServo.position = scaledValue
            field = value.coerceIn(0.0, 1.0)
        }

    init {
        position = 0.0
        realPosition = targetPosition
    }

    fun goToTargetNow() {
        realPosition = targetPosition
    }

    fun update(deltaTime: Double) {
        val step = speed * deltaTime
        val error = targetPosition - realPosition
        realPosition += if (abs(error) < step) {
            error
        } else {
            sign(error) * step
        }
    }
}