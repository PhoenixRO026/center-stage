package org.firstinspires.ftc.teamcode.systems.multi

import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.acmerobotics.roadrunner.Action
import com.qualcomm.robotcore.hardware.ColorRangeSensor
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.NormalizedRGBA
import org.firstinspires.ftc.teamcode.lib.units.Time
import org.firstinspires.ftc.teamcode.lib.units.ms
import org.firstinspires.ftc.teamcode.lib.units.s
import org.firstinspires.ftc.teamcode.systems.multi.Color2Multi.ColorConfig.afterYellowWaitSec
import org.firstinspires.ftc.teamcode.systems.multi.Color2Multi.ColorConfig.greenLowerThreshold
import org.firstinspires.ftc.teamcode.systems.multi.Color2Multi.ColorConfig.greenUpperThreshold
import org.firstinspires.ftc.teamcode.systems.multi.Color2Multi.ColorConfig.maxWaitSec
import org.firstinspires.ftc.teamcode.systems.multi.Color2Multi.ColorConfig.redLowerThreshold
import org.firstinspires.ftc.teamcode.systems.multi.Color2Multi.ColorConfig.redUpperThreshold

class Color2Multi(
    hardwareMap: HardwareMap
) {
    companion object {
        fun HardwareMap.color2Multi() = Color2Multi(this)
    }

    @Config
    data object ColorConfig {
        @JvmField var redLowerThreshold = 0.012
        @JvmField var greenLowerThreshold = 0.02
        @JvmField var redUpperThreshold = 0.026
        @JvmField var greenUpperThreshold = 0.04
        @JvmField var afterYellowWaitSec = 0.05
        @JvmField var maxWaitSec = 1.0
    }

    private val colorSens = hardwareMap.get(ColorRangeSensor::class.java, "color")

    private var freshRead = false

    var readingEnabled = false

    private var innerColor: NormalizedRGBA = colorSens.normalizedColors

    val color by ::innerColor

    val yellowPixelIn get() = color.green >= greenLowerThreshold && color.red >= redLowerThreshold && color.green <= greenUpperThreshold && color.red <= redUpperThreshold

    fun waitTillYellow() = object : Action {
        var init = true
        var initTime = 0.s
        var yellowReadTime = 0.ms
        var yellowInit = true

        override fun run(p: TelemetryPacket): Boolean {
            if (init) {
                init = false
                freshRead = false
                readingEnabled = true
                initTime = Time.now()
            }

            if (Time.now() > initTime + maxWaitSec.s) {
                readingEnabled = false
                return false
            }

            if (freshRead && yellowPixelIn) {
                if (yellowInit) {
                    yellowInit = false
                    yellowReadTime = Time.now()
                }
                if (Time.now() > yellowReadTime + afterYellowWaitSec.s) {
                    readingEnabled = false
                    return false
                }
            }

            return true
        }
    }

    fun read() {
        if (readingEnabled) {
            freshRead = true
            innerColor = colorSens.normalizedColors
        }
    }
}