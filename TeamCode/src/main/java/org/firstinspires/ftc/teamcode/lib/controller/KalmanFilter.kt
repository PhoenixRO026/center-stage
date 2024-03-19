package org.firstinspires.ftc.teamcode.lib.controller

import org.firstinspires.ftc.teamcode.lib.controller.utils.LinearRegression
import org.firstinspires.ftc.teamcode.lib.controller.utils.SizedStack
import org.firstinspires.ftc.teamcode.lib.controller.utils.doubleArray


/**
 * A kalman filter that uses a least squares regression as it's model.
 * @param Q Sensor Covariance
 * @param R Model Covariance
 * @param N Number of elements we can hold in our stack.
 */
data class KalmanFilter(
    @JvmField var Q: Double = 0.1,
    @JvmField var R: Double = 0.4,
    @JvmField var N: Int = 3
) {
    private val estimates = SizedStack(N, 0.0)
    private var regression = LinearRegression(estimates.doubleArray())

    private var P = 1.0
    private var K = 0.0

    var x = 0.0

    private var lastQ = Q
    private var lastR = R

    init {
        findK()
    }

    fun estimate(measurement: Double): Double {
        updateK()
        regression.runLeastSquares()
        x += regression.predictNextValue() - estimates.peek()
        x += K * (measurement - x)
        estimates.push(x)
        regression = LinearRegression(estimates.doubleArray())
        return x
    }


    private fun updateK() {
        if (lastQ != Q || lastR != R) {
            lastQ = Q
            lastR = R
            P = 1.0
            K = 0.0
            findK()
        }
    }

    private fun findK() {
        for (i in 1..2000) solveDARE()
    }

    private fun solveDARE() {
        P += Q
        K = P / (P + R)
        P *= (1 - K)
    }
}