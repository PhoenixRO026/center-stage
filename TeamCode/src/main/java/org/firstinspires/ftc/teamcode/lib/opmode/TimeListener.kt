package org.firstinspires.ftc.teamcode.lib.opmode

import org.firstinspires.ftc.teamcode.lib.units.Time

interface TimeListener {
    val deltaTime: Time
    val elapsedTime: Time
}