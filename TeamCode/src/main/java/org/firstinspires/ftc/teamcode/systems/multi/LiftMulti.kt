package org.firstinspires.ftc.teamcode.systems.multi

import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.acmerobotics.roadrunner.Action
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.teamcode.lib.multi.LazyWrite
import org.firstinspires.ftc.teamcode.systems.Intake.Companion.intake
import org.firstinspires.ftc.teamcode.systems.Lift
import org.firstinspires.ftc.teamcode.systems.Lift.Companion.lift
import java.util.concurrent.ConcurrentLinkedQueue

class LiftMulti(
        private val innerLift: Lift
) {
    companion object {
        fun HardwareMap.liftMulti() = LiftMulti(this.lift())
    }

    private val queue = ConcurrentLinkedQueue<LazyWrite.Task>()

    var mode by innerLift::mode

    var power by LazyWrite(
            queue,
            { innerLift.power = it },
            { innerLift.power }
    )

    private var innerPositionTicks = innerLift.positionTicks

    val positionTicks by ::innerPositionTicks

    var targetPositionTicks by innerLift::targetPositionTicks

    private var innerIsBusy = false

    val isBusy by ::innerIsBusy

    fun goToPos(pos: Int) = object : Action {
        var init = true

        override fun run(p: TelemetryPacket): Boolean {
            if (init) {
                init = false
                mode = Lift.MODE.TARGET
                targetPositionTicks = pos
                innerIsBusy = true
            }

            return isBusy
        }
    }

    fun read() {
        innerPositionTicks = innerLift.positionTicks
        innerIsBusy = innerLift.isBusy
    }

    fun write() {
        var task: LazyWrite.Task? = queue.poll()

        while (task != null) {
            task.task()
            task = queue.poll()
        }
    }

    fun update() {
        innerLift.update()
    }
}