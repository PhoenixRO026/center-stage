package org.firstinspires.ftc.teamcode.sensor

import com.ThermalEquilibrium.homeostasis.Filters.FilterAlgorithms.KalmanFilter
import com.acmerobotics.dashboard.config.Config
import com.qualcomm.hardware.rev.Rev2mDistanceSensor
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.firstinspires.ftc.teamcode.units.Distance
import org.firstinspires.ftc.teamcode.units.mm

class KalmanDistanceSensor(private val sensor: Rev2mDistanceSensor) {
    @Config
    data object KalmanConfig {
        //Q = how much we trust the sensor
        @JvmField var Q: Double = 0.3
        //R = how much we trust the linear regression
        @JvmField var R: Double = 3.0
        //N = number of elements we perform regression on
        @JvmField var N: Int = 3
        private var prevQ = Q
        private var prevR = R
        private var prevN = N
        val hasChanged get() = prevQ != Q || prevR != R || prevN != N
    }
    private var filter = KalmanFilter(KalmanConfig.Q, KalmanConfig.R, KalmanConfig.N)
    val position: Distance get() {
        if (KalmanConfig.hasChanged) {
            filter = KalmanFilter(KalmanConfig.Q, KalmanConfig.R, KalmanConfig.N)
        }
        val input = sensor.getDistance(DistanceUnit.MM)
        val estimate = filter.estimate(input)
        return estimate.mm
    }
    val rawPosition get() = sensor.getDistance(DistanceUnit.MM).mm
}