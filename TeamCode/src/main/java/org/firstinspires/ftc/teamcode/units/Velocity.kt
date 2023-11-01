package org.firstinspires.ftc.teamcode.units

class Velocity(val distance: Distance, val duration: Duration) {

    fun valueAs(velocity: Velocity): Double {
        val distanceVal = when (velocity.distance) {
            is Centimeters -> distance.toCentimeters()
            is Inches -> distance.toInches()
            is Meters -> distance.toMeters()
            is Milimeters -> distance.toMilimeters()
        }

        val durationVal = when (velocity.duration) {
            is Seconds -> duration.toSeconds()
            is Minutes -> duration.toMinutes()
        }

        return distanceVal / durationVal
    }
}

fun Distance.per(duration: Duration): Velocity {
    return Velocity(this, duration)
}