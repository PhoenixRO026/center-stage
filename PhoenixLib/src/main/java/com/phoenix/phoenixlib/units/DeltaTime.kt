package com.phoenix.phoenixlib.units

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