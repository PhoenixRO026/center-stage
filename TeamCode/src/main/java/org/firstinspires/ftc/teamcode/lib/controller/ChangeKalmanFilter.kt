package org.firstinspires.ftc.teamcode.lib.controller

import com.acmerobotics.roadrunner.Pose2d
import com.acmerobotics.roadrunner.Twist2d
import com.acmerobotics.roadrunner.Vector2d


class ChangeKalmanFilter {
    /**
     * @param Q model covariance (trust in model), default 0.1
     * @param R sensor covariance (trust in sensor), default 0.4
     */
    data class KalmanCoefficients @JvmOverloads constructor(
        val Q: Double = 0.1,
        val R: Double = 0.4
    )

    var x = 0.0 // your initial state
    private var p = 1.0 // your initial covariance guess
    private var K = 1.0 // your initial Kalman gain guess
    private val kalman: KalmanCoefficients

    /**
     * @param Q model covariance (trust in model), default 0.1
     * @param R sensor covariance (trust in sensor), default 0.4
     */
    @JvmOverloads
    constructor(Q: Double = 0.1, R: Double = 0.4) {
        kalman = KalmanCoefficients(Q, R)
        findK()
    }

    constructor(kalman: KalmanCoefficients = KalmanCoefficients()) {
        this.kalman = kalman
        findK()
    }

    private var x_previous = x
    private var p_previous = p
    private var u = 0.0
    private var z = 0.0

    /** Run in your loop.
     *
     * @param model the CHANGE(!) in state from the model
     * @param sensor the state from the sensor
     * @return the kalman filtered state
     */
    fun update(model: Double, sensor: Double): Double {
        u = model // Ex: change in position from odometry.
        x = x_previous + u
        p = p_previous + kalman.Q
        K = p / (p + kalman.R)
        z = sensor // Pose Estimate from April Tag / Distance Sensor
        x += K * (z - x)
        p *= (1 - K)
        x_previous = x
        p_previous = p
        return x
    }

    private fun findK() {
        for (i in 1..2000) solveDARE()
    }

    private fun solveDARE() {
        p += kalman.Q
        K = p / (p + kalman.R)
        p *= (1 - K)
    }

    class Vector2dKalmanFilter {
        val x: ChangeKalmanFilter
        val y: ChangeKalmanFilter

        constructor(x: ChangeKalmanFilter = ChangeKalmanFilter(), y: ChangeKalmanFilter = ChangeKalmanFilter()) {
            this.x = x
            this.y = y
        }

        @JvmOverloads
        constructor(Q: Double = 0.1, R: Double = 0.4) {
            x = ChangeKalmanFilter(Q, R)
            y = ChangeKalmanFilter(Q, R)
        }

        /**
         * NOTE: MODEL IS THE *CHANGE* SINCE LAST UPDATE
         */
        fun update(model: Twist2d, sensor: Pose2d): Vector2d {
            val modelPose: Pose2d = Pose2d.exp(model)
            return Vector2d(
                x.update(modelPose.position.x, sensor.position.x),
                y.update(modelPose.position.y, sensor.position.y)
            )
        }

        /**
         * NOTE: MODEL IS THE *CHANGE* SINCE LAST UPDATE
         * Probably don't switch between using a vector and a pose sensor
         */
        fun update(model: Twist2d, sensor: Vector2d): Vector2d {
            return Vector2d(
                x.update(model.line.x, sensor.x),
                y.update(model.line.y, sensor.y)
            )
        }
    }
}