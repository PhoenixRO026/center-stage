package org.firstinspires.ftc.teamcode.robot

import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.acmerobotics.roadrunner.Action
import com.acmerobotics.roadrunner.InstantAction
import com.acmerobotics.roadrunner.SequentialAction
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.Servo
import org.firstinspires.ftc.teamcode.lib.units.SleepAction
import org.firstinspires.ftc.teamcode.lib.units.s
import org.firstinspires.ftc.teamcode.robot.hardware.ServoEx

class Plane(
        hardwareMap: HardwareMap
) {
    companion object {
        const val tiltRest = 0.351
        const val tiltLaunch = 0.493
        const val launchPlane = 0.13
        const val hold = 0.292

        fun HardwareMap.plane() = Plane(this)
    }

    private var launchAction: Action? = null

    private val tilt = ServoEx(
            servo = hardwareMap.get(Servo::class.java, "planeTilt"),
            changeTreshold = 0.0
    )
    private val launch = ServoEx(
            servo = hardwareMap.get(Servo::class.java, "plane"),
            changeTreshold = 0.0
    )

    var tiltPosition by tilt::position

    var launchPosition by launch::position

    fun launch() {
        launchAction = SequentialAction(
                InstantAction { tiltPosition = tiltLaunch },
                SleepAction(0.5.s),
                InstantAction { launchPosition = launchPlane },
                SleepAction(1.s),
                InstantAction {
                    tiltPosition = tiltRest
                    launchPosition = hold
                }
        )
    }

    fun update() {
        launchAction?.let {
            if (!it.run(TelemetryPacket())) {
                launchAction = null
            }
        }
    }

    init {
        tiltPosition = tiltRest
        launchPosition = hold
    }
}