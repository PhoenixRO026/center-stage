package org.firstinspires.ftc.teamcode.robot

import com.qualcomm.robotcore.hardware.ColorRangeSensor
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.firstinspires.ftc.teamcode.lib.units.mm

@Suppress("unused", "MemberVisibilityCanBePrivate")
class ColorSensorsMulti(
    hardwareMap: HardwareMap
) {
    companion object {
        fun colorSensMultiH(hardwareMap: HardwareMap) = ColorSensorsMulti(hardwareMap)

        fun HardwareMap.colorSensMulti() = ColorSensorsMulti(this)
    }

    //0 - V3, 2 - V2
    private val leftColorSens = hardwareMap.get(ColorRangeSensor::class.java, "leftColor")
    private val rightColorSens = hardwareMap.get(ColorRangeSensor::class.java, "rightColor")

    val leftPixelIn get() = leftColor.alpha > 0.75

    val rightPixelIn get() = rightColor.alpha > 0.045

    var leftColor = leftColorSens.normalizedColors
        private set

    var rightColor = rightColorSens.normalizedColors
        private set

    var leftDistance = leftColorSens.getDistance(DistanceUnit.MM).mm
        private set

    var rightDistance = rightColorSens.getDistance(DistanceUnit.MM).mm
        private set

    var leftLight = leftColorSens.lightDetected
        private set

    var rightLight = rightColorSens.lightDetected

    fun read() {
        leftColor = leftColorSens.normalizedColors
        leftDistance = leftColorSens.getDistance(DistanceUnit.MM).mm
        leftLight = leftColorSens.lightDetected
        rightColor = rightColorSens.normalizedColors
        rightDistance = rightColorSens.getDistance(DistanceUnit.MM).mm
        rightLight = rightColorSens.lightDetected
    }
}