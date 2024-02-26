package org.firstinspires.ftc.teamcode.lib.controller

import org.firstinspires.ftc.teamcode.lib.units.Time
import org.firstinspires.ftc.teamcode.lib.units.ms

@Suppress("unused")
data class PIDController(
    @JvmField var kP: Double = 0.0,
    @JvmField var kI: Double = 0.0,
    @JvmField var kD: Double = 0.0,
    @JvmField var integralLimit: Double = 0.0,
    @JvmField var integralReset: Boolean = true
) {
    private var previousTime = 0.ms
    private var init = true
    private var previousError = 0.0
    private var integralSumError = 0.0
    private var previousTarget = 0.0

    private fun deltaTime(): Time {
        if (init) {
            init = false
            previousTime = Time.now()
        }
        val now = Time.now()
        val dt = now - previousTime
        previousTime = now
        return dt
    }

    fun calculate(position: Double, target: Double): Double {
        val dt = deltaTime()

        val error = target - position

        integralSumError = when {
            integralReset && target != previousTarget -> 0.0
            integralLimit != 0.0 -> (integralSumError + (error + previousError) / 2 * dt.s).coerceIn(-integralLimit, integralLimit)
            else -> integralSumError + (error + previousError) / 2 * dt.s
        }
        previousTarget = target

        val derivativeError = (error - previousError) / dt.s

        previousError = error

        return error * kP + integralSumError * kI + derivativeError * kD
    }
}