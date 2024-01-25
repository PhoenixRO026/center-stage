package org.firstinspires.ftc.teamcode.lib.opmode

import org.firstinspires.ftc.teamcode.lib.units.Time
import org.firstinspires.ftc.teamcode.lib.units.ms

class TimeOpModeImpl : TimeListener {
    companion object {
        private var _deltaTimeProvider: () -> Time = { 0.ms }
        private var _elapsedTimeProvider: () -> Time = { 0.ms }

        fun setDeltaTimeProvider(newDeltaTimeProvider: () -> Time) {
            _deltaTimeProvider = newDeltaTimeProvider
        }

        fun setElapsedTimeProvider(newElapsedTimeProvider: () -> Time) {
            _elapsedTimeProvider = newElapsedTimeProvider
        }
    }

    override val deltaTime: Time
        get() = _deltaTimeProvider()

    override val elapsedTime: Time
        get() = _elapsedTimeProvider()
}