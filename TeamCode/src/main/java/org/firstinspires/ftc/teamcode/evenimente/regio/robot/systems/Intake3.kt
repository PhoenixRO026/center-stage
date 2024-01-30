package org.firstinspires.ftc.teamcode.evenimente.regio.robot.systems

import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.acmerobotics.roadrunner.Action
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.evenimente.alba.robot.CONFIG_ALBA
import org.firstinspires.ftc.teamcode.evenimente.beclean.robot.INTAKE_RANGE
import org.firstinspires.ftc.teamcode.evenimente.liga_wonder.robot.CONFIG
import org.firstinspires.ftc.teamcode.evenimente.liga_wonder.robot.hardware.MotorEx
import org.firstinspires.ftc.teamcode.evenimente.liga_wonder.robot.hardware.ServoEx

class Intake3(
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

    fun goToAngle(newAngle: Double) = object : Action {
        var init = true
        val elapsedTime = ElapsedTime()

        override fun run(p: TelemetryPacket): Boolean {
            if (init) {
                init = false
                angle = newAngle
                elapsedTime.reset()
            }

            return elapsedTime.seconds() < 0.2
        }

    }

    init {
        angle = 0.0
    }
}