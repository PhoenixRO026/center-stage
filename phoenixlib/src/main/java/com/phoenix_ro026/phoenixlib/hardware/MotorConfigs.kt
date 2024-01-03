package com.phoenix_ro026.phoenixlib.hardware

import com.qualcomm.hardware.motors.GoBILDA5202Series

val GOBILDA_312 = Motor.Config(
    Motor.Direction.COUNTER_CLOCK_WISE,
    (((1.0+(46.0/17.0))) * (1.0+(46.0/11.0))) * 28.0
    )