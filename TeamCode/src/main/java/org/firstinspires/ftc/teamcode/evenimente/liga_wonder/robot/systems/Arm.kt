package org.firstinspires.ftc.teamcode.evenimente.liga_wonder.robot.systems

import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.Servo
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.evenimente.liga_wonder.robot.CONFIG
import org.firstinspires.ftc.teamcode.evenimente.liga_wonder.robot.LEFT_ARM_SERVO_RANGE
import org.firstinspires.ftc.teamcode.evenimente.liga_wonder.robot.RIGHT_ARM_SERVO_RANGE
import org.firstinspires.ftc.teamcode.evenimente.liga_wonder.robot.hardware.ServoEx
import org.firstinspires.ftc.teamcode.evenimente.liga_wonder.robot.hardware.scaleTo

class Arm(
    hardwareMap: HardwareMap,
    private val telemetry: Telemetry? = null,
    val range: ClosedRange<Double> = 0.0..1.0
) {
    private val rightServo = ServoEx(hardwareMap, CONFIG.RIGHT_ARM, RIGHT_ARM_SERVO_RANGE, telemetry)
    private val leftServo = ServoEx(hardwareMap, CONFIG.LEFT_ARM, LEFT_ARM_SERVO_RANGE, telemetry)

    var position: Double = 0.0
        set(value) {
            val scaledValue = value.coerceIn(0.0, 1.0).scaleTo(range)
            rightServo.position = scaledValue
            leftServo.position = scaledValue
            field = value.coerceIn(0.0, 1.0)
        }

    init {
        position = 0.0
    }
}