package org.firstinspires.ftc.teamcode.evenimente.liga_wonder.robot.systems

import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.evenimente.liga_wonder.robot.CONFIG
import org.firstinspires.ftc.teamcode.evenimente.liga_wonder.robot.hardware.MotorEx

class Intake(
    hardwareMap: HardwareMap,
    private val telemetry: Telemetry? = null
) {
    private val motor = MotorEx(hardwareMap, CONFIG.INTAKE, telemetry)

    var power: Double = 0.0
        set(value) {
            motor.power = value
            field = value
        }
}