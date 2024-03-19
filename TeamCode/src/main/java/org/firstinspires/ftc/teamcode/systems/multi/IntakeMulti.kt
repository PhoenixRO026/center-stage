package org.firstinspires.ftc.teamcode.systems.multi

import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.teamcode.lib.multi.LazyWrite
import org.firstinspires.ftc.teamcode.systems.Arm.Companion.arm
import org.firstinspires.ftc.teamcode.systems.Intake
import org.firstinspires.ftc.teamcode.systems.Intake.Companion.intake
import java.util.concurrent.ConcurrentLinkedQueue

class IntakeMulti(
        private val innerIntake: Intake
) {
    companion object {
        fun HardwareMap.intakeMulti() = IntakeMulti(this.intake())
    }

    private val queue = ConcurrentLinkedQueue<LazyWrite.Task>()

    var position by LazyWrite(
            queue,
            { innerIntake.position = it },
            { innerIntake.position }
    )

    var targetPosition by innerIntake::targetPosition

    var power by LazyWrite(
            queue,
            { innerIntake.power = it },
            { innerIntake.power }
    )

    val isBusy by innerIntake::isBusy

    fun goDown() {
        position -= innerIntake.pubDeltaTime.s
    }

    fun goUp() {
        position += innerIntake.pubDeltaTime.s
    }

    fun write() {
        var task: LazyWrite.Task? = queue.poll()

        while (task != null) {
            task.task()
            task = queue.poll()
        }
    }

    fun update() {
        innerIntake.update()
    }
}