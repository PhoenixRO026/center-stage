package org.firstinspires.ftc.teamcode.evenimente.beclean.robot.systems

import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.evenimente.beclean.robot.PLANE_RANGE
import org.firstinspires.ftc.teamcode.evenimente.liga_wonder.robot.CONFIG
import org.firstinspires.ftc.teamcode.evenimente.liga_wonder.robot.hardware.ServoEx

class Plane(
    hardwareMap: HardwareMap,
    private val telemetry: Telemetry? = null
) {
    private val servo = ServoEx(hardwareMap, CONFIG.PLANE, PLANE_RANGE)

    var position = 1.0
        set(value) {
            val scaledValue = value.coerceIn(0.0, 1.0)
            servo.position = scaledValue
            field = scaledValue
        }

    init {
        position = 1.0
    }

    fun launch() {
        position = 0.0
    }
}