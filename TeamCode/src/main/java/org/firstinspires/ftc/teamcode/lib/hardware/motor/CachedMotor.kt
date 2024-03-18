package org.firstinspires.ftc.teamcode.lib.hardware.motor

import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.DcMotorSimple.Direction
import com.qualcomm.robotcore.hardware.HardwareMap
import kotlin.math.abs
import kotlin.math.sign

class CachedMotor @JvmOverloads constructor(
    dcMotor: DcMotorSimple,
    direction: Direction = Direction.FORWARD,
    @JvmField val cachingThreshold: Double = defaultCachingThreshold,
    coupledMotors: List<SimpleMotor> = emptyList()
) : SimpleMotor(dcMotor, direction, coupledMotors) {
    companion object {
        const val defaultCachingThreshold = 0.01

        fun HardwareMap.cachedMotor(
            deviceName: String,
            direction: Direction = Direction.FORWARD,
            cachingThreshold: Double = defaultCachingThreshold,
            coupledMotors: List<SimpleMotor> = emptyList()
        ) = CachedMotor(this, deviceName, direction, cachingThreshold, coupledMotors)
    }

    @JvmOverloads constructor(
        hardwareMap: HardwareMap,
        deviceName: String,
        direction: Direction = Direction.FORWARD,
        cachingThreshold: Double = defaultCachingThreshold,
        coupledMotors: List<SimpleMotor> = emptyList()
    ) : this (hardwareMap.get(DcMotorSimple::class.java, deviceName), direction, cachingThreshold, coupledMotors)

    private var innerPower
        get() = super.power
        set(value) {
            super.power = value
        }

    override var power: Double = 0.0
        set(value) {
            val newValue = value.coerceIn(-1.0, 1.0)
            if (shouldChangePower(newValue)) {
                innerPower = newValue
            }
            field = newValue
        }

    fun setPowerResult(power: Double): Boolean {
        val pow = power.coerceIn(0.0, 1.0)
        val result = shouldChangePower(pow)
        this.power = pow
        return result
    }

    fun shouldChangePower(power: Double): Boolean {
        val newPower = power.coerceIn(-1.0, 1.0)
        val overChangeThreshold = abs(newPower - innerPower) >= cachingThreshold
        val targetingFullPower = (newPower >= 1.0 && innerPower < 1.0) || (newPower <= -1.0 && innerPower > -1.0)
        val changedDirectionOrBrake = newPower.sign != innerPower.sign
        return overChangeThreshold || targetingFullPower || changedDirectionOrBrake
    }
}