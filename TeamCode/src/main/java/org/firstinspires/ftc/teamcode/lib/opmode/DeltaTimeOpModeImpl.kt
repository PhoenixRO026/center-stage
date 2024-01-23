package org.firstinspires.ftc.teamcode.lib.opmode

import org.firstinspires.ftc.teamcode.lib.units.Time
import org.firstinspires.ftc.teamcode.lib.units.ms

class DeltaTimeOpModeImpl : DeltaTimeListener {
    private var _deltaTime = 0.ms

    override val deltaTime: Time
        get() = _deltaTime

    fun updateDeltaTime(newDeltaTime: Time) {
        _deltaTime = newDeltaTime
    }
}