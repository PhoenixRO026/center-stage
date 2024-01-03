package com.phoenix_ro026.phoenixlib.hardware

import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType

class Motor(hardwareMap: HardwareMap, deviceName: String, val config: Config) {
    private val motor: DcMotorEx = hardwareMap.get(DcMotorEx::class.java, deviceName)

    var direction: Direction = config.defaultDirection
        set(value) {
            if (value == config.defaultDirection) {
                motor.direction = DcMotorSimple.Direction.FORWARD
            } else {
                motor.direction = DcMotorSimple.Direction.REVERSE
            }
            field = value
        }

    var power: Number = 0
        set(value) {
            motor.power = value.toDouble()
            field = value
        }

    init {

    }

    data class Config(
        val defaultDirection: Direction,
        val internalGearRatio: Double
    )

    enum class Direction {
        CLOCK_WISE,
        COUNTER_CLOCK_WISE
    }
}