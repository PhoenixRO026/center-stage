package org.firstinspires.ftc.teamcode.evenimente.alba.robot

import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.evenimente.liga_wonder.robot.CONFIG
import org.firstinspires.ftc.teamcode.evenimente.liga_wonder.robot.hardware.MotorEx
import org.firstinspires.ftc.teamcode.evenimente.liga_wonder.robot.hardware.ServoEx

class Intake2(
    hardwareMap: HardwareMap,
    private val telemetry: Telemetry? = null
) {
    private val motor = MotorEx(hardwareMap, CONFIG.INTAKE, telemetry)
    private val servo = ServoEx(hardwareMap, CONFIG_ALBA.INTAKE_SERVO, INTAKE_RANGE)

    var power: Double = 0.0
        set(value) {
            motor.power = value
            field = value
        }

    var angle: Double = 0.0
        set(value) {
            servo.position = value.coerceIn(0.0, 1.0)
            field = value.coerceIn(0.0, 1.0)
        }

    init {
        angle = 0.0
    }
}