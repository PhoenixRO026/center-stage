package org.firstinspires.ftc.teamcode.evenimente.code_camp.robot.feedforward

import kotlin.math.cos
import kotlin.math.sign

class ArmFeedforward @JvmOverloads constructor(
    private var ks: Double = 0.0,
    private var kcos: Double = 0.0,
    private var kv: Double = 0.0,
    private var ka: Double = 0.0,
) {
    @JvmOverloads
    fun calculate(
        positionRadians: Double,
        velocityRadPerSec: Double,
        voltage: Double = 12.0,
        accelRadPerSecSquared: Double = 0.0
    ): Double {
        return  ks * sign(velocityRadPerSec) * (12.0 / voltage) +
                kcos * cos(positionRadians) * (12.0 / voltage) +
                kv * velocityRadPerSec +
                ka * accelRadPerSecSquared
    }

    @JvmOverloads
    fun updateCoeficients(
        ks: Double = 0.0,
        kcos: Double = 0.0,
        kv: Double = 0.0,
        ka: Double = 0.0,
    ) {
        this.ks = ks
        this.kcos = kcos
        this.kv = kv
        this.ka = ka
    }
}