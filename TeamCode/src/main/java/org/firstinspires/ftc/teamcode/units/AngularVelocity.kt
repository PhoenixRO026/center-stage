package org.firstinspires.ftc.teamcode.units

class AngularVelocity(val rotation: Rotation, val duration: Duration) {

    infix fun convertedAs(angularVelocity: AngularVelocity): Double {
        val rotationVal = when (angularVelocity.rotation) {
            is Degrees -> rotation.toDegrees()
            is Radians -> rotation.toRadians()
            is Revolutions -> rotation.toRevolutions()
        }

        val durationVal = when (angularVelocity.duration) {
            is Seconds -> duration.toSeconds()
            is Minutes -> duration.toMinutes()
        }

        return rotationVal / durationVal
    }
}

fun Rotation.per(duration: Duration): AngularVelocity {
    return AngularVelocity(this, duration)
}

/*
fun test() {
    10.degrees.per(2.minutes) convertedAs radian.per(second)
}*/
