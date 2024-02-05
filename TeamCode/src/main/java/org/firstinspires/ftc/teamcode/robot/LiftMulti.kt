package org.firstinspires.ftc.teamcode.robot

import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.acmerobotics.roadrunner.Action
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.teamcode.lib.units.deg
import java.util.concurrent.ConcurrentLinkedQueue

@Suppress("unused", "MemberVisibilityCanBePrivate")
class LiftMulti(
    hardwareMap: HardwareMap,
    private val innerLift: Lift = Lift(hardwareMap)
) {
    companion object {
        fun liftMultiH(hardwareMap: HardwareMap) = LiftMulti(hardwareMap)

        fun HardwareMap.liftMulti() = LiftMulti(this)
    }

    private val queue = ConcurrentLinkedQueue<() -> Unit>()

    var power by LazyWrite(
        queue,
        { innerLift.power = it },
        { innerLift.power }
    )

    var position = 0
        private set

    var angle = 0.deg
        private set

    var isBusy = false
        private set

    fun goToTicksAsync(ticks: Int) {
        queue.add {
            innerLift.goToTicksAsync(ticks)
        }
    }

    fun goToTicks(ticks: Int) = object : Action {
        var init = true

        override fun run(p: TelemetryPacket): Boolean {
            if (init) {
                init = false
                goToTicksAsync(ticks)
                isBusy = true
            }

            return isBusy
        }
    }
    fun goToRampAsync() {
        queue.add {
            innerLift.goToRampAsync()
        }
    }

    fun hangAsync() {
        queue.add {
            innerLift.hangAsync()
        }
    }

    fun unhangAsync() {
        queue.add {
            innerLift.unhangAsync()
        }
    }

    fun read() {
        position = innerLift.position
        angle = innerLift.angle
        isBusy = innerLift.isBusy
    }

    fun write() {
        while (queue.isNotEmpty()) {
            val action = queue.remove()
            action()
        }
    }
}