package org.firstinspires.ftc.teamcode.lib.action

import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.acmerobotics.roadrunner.Action

data class DependendAction(
    val action: Action,
    val dependendFunction: () -> Unit
) : Action {
    override fun run(p: TelemetryPacket): Boolean {
        dependendFunction()

        return action.run(p)
    }
}