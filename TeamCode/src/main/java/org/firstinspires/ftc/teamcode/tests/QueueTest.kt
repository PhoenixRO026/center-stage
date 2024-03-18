package org.firstinspires.ftc.teamcode.tests

import com.arcrobotics.ftclib.gamepad.ButtonReader
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.lib.multi.LazyWrite
import org.firstinspires.ftc.teamcode.lib.opmode.MultiThreadOpMode
import java.util.concurrent.ConcurrentLinkedQueue

@TeleOp
class QueueTest : MultiThreadOpMode() {

    private val queue = ConcurrentLinkedQueue<LazyWrite.Task>()

    @Volatile
    private var counter = 0

    private var fakePosition1 by LazyWrite(
        queue,
        { counter += it },
        { 2 }
    )

    private var fakePosition2 by LazyWrite(
        queue,
        { counter -= it },
        { 1 }
    )

    override fun sideRunOpMode() {
        val buttonY = ButtonReader { gamepad1.y }

        waitForStart()

        while (opModeIsActive()) {
            buttonY.readValue()

            if (buttonY.wasJustPressed()) {
                queue.poll()?.let { it.task() }
            }
        }
    }

    override fun mainRunOpMode() {
        val buttonA = ButtonReader { gamepad1.a }
        val buttonB = ButtonReader { gamepad1.b }

        waitForStart()

        while (opModeIsActive()) {
            buttonA.readValue()
            buttonB.readValue()

            if (buttonA.wasJustPressed()) {
                fakePosition1 = 3
            }

            if (buttonB.wasJustPressed()) {
                fakePosition2 = 2
            }

            queue.forEachIndexed { index, task ->
                telemetry.addLine("index $index with tash time ${task.time}")
            }

            telemetry.addLine("counter: $counter")

            telemetry.update()
        }
    }
}