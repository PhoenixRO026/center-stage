package org.firstinspires.ftc.teamcode.stc

import org.firstinspires.ftc.teamcode.lib.units.Time
import org.firstinspires.ftc.teamcode.lib.units.ms

class DeltaTime {
    private var previousTime: Time? = null

    fun calculateDeltaTime(): Time {
        val now = System.currentTimeMillis().ms
        val deltaTime = previousTime?.let { prevTime ->
            now - prevTime
        } ?: 1.ms
        previousTime = now
        return deltaTime
    }
}