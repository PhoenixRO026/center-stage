package org.firstinspires.ftc.teamcode.stc

import kotlin.math.absoluteValue
import kotlin.math.sign

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
    private var previousTarget = 0.0

    private val deltaTime = DeltaTime()

    private var innerIntegralSum = 0.0
    private var innerDerivative = 0.0

    val derivative by ::innerDerivative
    val integralSum by ::innerIntegralSum

    val derivativeKD get() = derivative * kD
    val integralSumKI get() = integralSum * kI

    fun calculate(position: Double, target: Double): Double {
        val dt = deltaTime.calculateDeltaTime()

        val error = target - position

        innerDerivative = calculateDerivative(error, dt.s)

        integrate(error, target, dt.s, derivative)

        previousTarget = target

        previousError = error

        return error * kP + integralSum * kI + derivative * kD
    }

    private fun integrate(error: Double, target: Double, dt: Double, derivative: Double) {
        if (crossOverDetected(error)) {
            innerIntegralSum = 0.0
        }

        if (newTargetReset && target != previousTarget) {
            innerIntegralSum = 0.0
        }

        if (stabilityThreshold > 0.0 && derivative.absoluteValue > stabilityThreshold) return

        innerIntegralSum += ((error + previousError) / 2) * dt

        if (integralSumLimit > 0) {
            innerIntegralSum = innerIntegralSum.coerceIn(-integralSumLimit, integralSumLimit)
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
        innerIntegralSum = 0.0
    }
}