package org.firstinspires.ftc.teamcode.lib.controller

import org.firstinspires.ftc.teamcode.lib.units.DeltaTime
import org.firstinspires.ftc.teamcode.lib.units.ms
import kotlin.math.absoluteValue
import kotlin.math.sign

/**
 * @param kP proportional term
 * @param kI integral term
 * @param kD derivative term
 * @param integralSumLimit the maximum our integral sum can go to
 *                              (this * Kp should be less than the maximum output of your system)
 * @param newTargetReset should the system reset the integral sum if the target changes
 * @param derivativeFilter low pass filter for the derivative
 * @param stabilityThreshold the maximum our derivative can be before we integrate.
 *                           This ensures we have better stability
 */
@Suppress("unused")
data class PIDController(
    @JvmField var kP: Double = 0.0,
    @JvmField var kI: Double = 0.0,
    @JvmField var kD: Double = 0.0,
    @JvmField var integralSumLimit: Double = 0.0,
    @JvmField var newTargetReset: Boolean = true,
    @JvmField var derivativeFilter: LowPassFilter = LowPassFilter(),
    @JvmField var stabilityThreshold: Double = 0.0
) {
    private var previousTime = 0.ms
    private var init = true
    private var previousError = 0.0
    private var integralSumError = 0.0
    private var previousTarget = 0.0

    private val deltaTime = DeltaTime()

    fun calculate(position: Double, target: Double): Double {
        val dt = deltaTime.calculateDeltaTime()

        val error = target - position

        val derivativeError = calculateDerivative(error, dt.s)

        integrate(error, target, dt.s, derivativeError)

        previousTarget = target

        previousError = error

        return error * kP + integralSumError * kI + derivativeError * kD
    }

    private fun integrate(error: Double, target: Double, dt: Double, derivative: Double) {
        if (crossOverDetected(error)) {
            integralSumError = 0.0
        }

        if (newTargetReset && target != previousTarget) {
            integralSumError = 0.0
        }

        if (stabilityThreshold > 0.0 && derivative.absoluteValue > stabilityThreshold) return

        integralSumError += ((error + previousError) / 2) * dt

        if (integralSumLimit > 0) {
            integralSumError = integralSumError.coerceIn(-integralSumLimit, integralSumLimit)
        }
    }

    private fun calculateDerivative(error: Double, dt: Double): Double {
        val derivative = (error - previousError) / dt
        return derivativeFilter.estimate(derivative)
    }

    private fun crossOverDetected(error: Double): Boolean {
        return error.sign != previousError.sign && error.sign != 0.0
    }

    fun resetIntegral() {
        integralSumError = 0.0
    }
}