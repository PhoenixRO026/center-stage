package org.firstinspires.ftc.teamcode.systems.multi

import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.acmerobotics.roadrunner.Action
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.teamcode.lib.multi.LazyWrite
import org.firstinspires.ftc.teamcode.systems.Arm.Companion.arm
import org.firstinspires.ftc.teamcode.systems.Box
import org.firstinspires.ftc.teamcode.systems.Box.Companion.box
import java.util.concurrent.ConcurrentLinkedQueue

class BoxMulti(
        private val innerBox: Box
) {
    companion object {
        fun HardwareMap.boxMulti() = BoxMulti(this.box())
    }

    private val queue = ConcurrentLinkedQueue<LazyWrite.Task>()

    var position by LazyWrite(
            queue,
            { innerBox.position = it },
            { innerBox.position }
    )

    var targetPosition by innerBox::targetPosition

    var power by LazyWrite(
            queue,
            { innerBox.power = it },
            { innerBox.power }
    )

    val isBusy by innerBox::isBusy


    fun goToIntake() = object : Action {
        var init = true
        override fun run(p: TelemetryPacket): Boolean {
            if (init) {
                init = false
                innerBox.intakePos()
            }

            return isBusy
        }
    }

    fun goToScore() = object : Action {
        var init = true
        override fun run(p: TelemetryPacket): Boolean {
            if (init) {
                init = false
                innerBox.scorePos()
            }

            return isBusy
        }
    }

    fun write() {
        var task: LazyWrite.Task? = queue.poll()

        while (task != null) {
            task.task()
            task = queue.poll()
        }
    }

    fun update() {
        innerBox.update()
    }
}