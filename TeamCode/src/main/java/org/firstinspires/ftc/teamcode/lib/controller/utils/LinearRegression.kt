package org.firstinspires.ftc.teamcode.lib.controller.utils

import java.util.Arrays
import kotlin.math.pow


class LinearRegression(
    val y: DoubleArray
) {

    val x: DoubleArray = DoubleArray(y.size)
    private var m = 0.0
    private var b = 0.0

    init {
        for (i in x.indices) {
            x[i] = i.toDouble()
        }
    }

    fun runLeastSquares() {
        val n = x.size.toDouble()
        var xySum = 0.0
        for (i in x.indices) {
            xySum += x[i] * y[i]
        }
        val m1 = n * xySum - Arrays.stream(x).sum() * Arrays.stream(y).sum()
        var xSquaredsum = 0.0
        for (v in x) {
            xSquaredsum += v.pow(2.0)
        }
        val m2: Double = n * xSquaredsum - Arrays.stream(x).sum().pow(2.0)
        m = m1 / m2
        b = Arrays.stream(y).sum() - m * Arrays.stream(x).sum()
        b /= n
    }


    fun predictNextValue(): Double {
        println("in predict value, length is " + x.size + " m is " + m + " and b is " + b)
        return x.size * m + b
    }

}