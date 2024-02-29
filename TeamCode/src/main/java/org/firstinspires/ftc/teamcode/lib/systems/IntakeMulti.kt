@file:Suppress("unused")

package org.firstinspires.ftc.teamcode.lib.systems

import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.acmerobotics.roadrunner.Action
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.teamcode.lib.systems.Intake.Companion.intake
import org.firstinspires.ftc.teamcode.lib.units.Time
import java.util.concurrent.ConcurrentLinkedQueue
import kotlin.math.abs

class IntakeMulti(
    hardwareMap: HardwareMap,
    private val innerIntake: Intake = hardwareMap.intake()
) {
    companion object {
        fun intakeMultiH(hardwareMap: HardwareMap) = IntakeMulti(hardwareMap)

        fun HardwareMap.intakeMulti() = IntakeMulti(this)
    }

    private val queue = ConcurrentLinkedQueue<() -> Unit>()

    var power by LazyWrite(
        queue,
        { innerIntake.power = it },
        { innerIntake.power }
    )

    var position by LazyWrite(
        queue,
        { innerIntake.position = it },
        { innerIntake.position }
    )

    var targetPosition by innerIntake::targetPosition

    var speed by innerIntake::speed

    val isBusy by innerIntake::isBusy

    fun waitForPos(newPos: Double) = object : Action {
        var init = true

        override fun run(p: TelemetryPacket): Boolean {
            if (init) {
                init = false

                position = newPos
            }

            return abs(position - newPos) > 0.001
        }
    }

    fun goToPos(newPos: Double) = object : Action {
        var init = true

        override fun run(p: TelemetryPacket): Boolean {
            if (init) {
                init = false

                targetPosition = newPos
            }

            return isBusy
        }
    }

    fun update(deltaTime: Time) {
        innerIntake.update(deltaTime)
    }

    fun write() {
        while (queue.isNotEmpty()) {
            val action = queue.remove()
            action()
        }
    }
}