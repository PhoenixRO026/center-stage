package com.phoenix_ro026.phoenixlib.opmode

import com.phoenix_ro026.phoenixlib.hardware.Motor
import com.qualcomm.robotcore.hardware.HardwareMap

class HardwareMapEx(val hardwareMap: HardwareMap) {
    fun getMotor(deviceName: String, config: Motor.Config) = Motor(hardwareMap, deviceName, config)
}