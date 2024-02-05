package org.firstinspires.ftc.teamcode.robot

import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.acmerobotics.roadrunner.Action
import com.acmerobotics.roadrunner.SequentialAction
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.teamcode.lib.units.Angle
import org.firstinspires.ftc.teamcode.lib.units.SleepAction
import org.firstinspires.ftc.teamcode.lib.units.Time
import org.firstinspires.ftc.teamcode.lib.units.s
import java.util.concurrent.ConcurrentLinkedQueue

@Suppress("unused", "MemberVisibilityCanBePrivate")
class ArmMulti(
    hardwareMap: HardwareMap,
    private val innerArm: Arm = Arm(hardwareMap)
) {
    private val queue: ConcurrentLinkedQueue<() -> Unit> = ConcurrentLinkedQueue()

    companion object {
        fun armMultiH(hardwareMap: HardwareMap) = ArmMulti(hardwareMap)

        fun HardwareMap.armMulti() = ArmMulti(this)
    }

    var position by LazyWrite(
        queue,
        { innerArm.position = it },
        { innerArm.position }
    )

    var targetPosition by innerArm::targetPosition

    var angle by LazyWrite(
        queue,
        { innerArm.angle = it },
        { innerArm.angle }
    )

    var targetAngle by innerArm::targetAngle

    val isBusy by innerArm::isBusy

    fun goToRamp() = goToPos(Arm.rampPos)

    fun goToPos(newPos: Double) = SequentialAction(
        object : Action {
            var init = true

            override fun run(p: TelemetryPacket): Boolean {
                if (init) {
                    init = false
                    targetPosition = newPos
                }

                return isBusy
            }
        },
        SleepAction(0.1.s)
    )

    fun goToAngle(newAngle: Angle) = SequentialAction(
        object : Action {
            var init = true

            override fun run(p: TelemetryPacket): Boolean {
                if (init) {
                    init = false
                    targetAngle = newAngle
                }

                return isBusy
            }
        },
        SleepAction(0.1.s)
    )

    fun update(deltaTime: Time) {
        innerArm.update(deltaTime)
    }

    fun write() {
        while (queue.isNotEmpty()) {
            val action = queue.remove()
            action()
        }
    }
}