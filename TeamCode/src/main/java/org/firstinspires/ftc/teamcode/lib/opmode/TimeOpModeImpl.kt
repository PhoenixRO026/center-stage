package org.firstinspires.ftc.teamcode.lib.opmode

import org.firstinspires.ftc.teamcode.lib.units.Time
import org.firstinspires.ftc.teamcode.lib.units.ms

class TimeOpModeImpl : TimeListener {
    companion object {
        private var _deltaTime = 0.ms
        private var _elapsedTime = 0.ms
        fun setDeltaTime(newDeltaTime: Time) {
            _deltaTime = newDeltaTime
        }

        fun setElapsedTime(newElapsedTime: Time) {
            _elapsedTime = newElapsedTime
        }
    }

    override val deltaTime by Companion::_deltaTime

    override val elapsedTime by Companion::_elapsedTime
}