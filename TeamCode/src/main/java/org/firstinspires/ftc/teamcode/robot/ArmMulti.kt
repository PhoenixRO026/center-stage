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

class ArmMulti(
    hardwareMap: HardwareMap,
    private val innerArm: Arm = Arm(hardwareMap)
) {
    val queue: ConcurrentLinkedQueue<() -> Unit> = ConcurrentLinkedQueue()

    companion object {
        fun armMulti(hardwareMap: HardwareMap) = ArmMulti(hardwareMap)

        fun HardwareMap.armMulti() = ArmMulti(this)
    }

    /*private var positionCache = LazyWrite(
        writeT = { newValue -> innerArm.position = newValue },
        getValueT = { innerArm.position }
    )

    var position by positionCache*/

    var position
        set(value) {
            queue.add {
                innerArm.position = value
            }
        }
        get() = innerArm.position

    /*private var targetPositionCache = LazyWrite(
        writeT = { newValue -> innerArm.targetPosition = newValue },
        getValueT = { innerArm.targetPosition }
    )

    var targetPosition by targetPositionCache*/

    var targetPosition by innerArm::targetPosition

    /*private var angleCache = LazyWrite(
        writeT = { newValue -> innerArm.angle = newValue },
        getValueT = { innerArm.angle }
    )

    var angle by angleCache*/

    var angle
        set(value) {
            queue.add {
                innerArm.angle = value
            }
        }
        get() = innerArm.angle

    /*private var targetAngleCache = LazyWrite(
        writeT = { newValue -> innerArm.targetAngle = newValue },
        getValueT = { innerArm.targetAngle }
    )

    var targetAngle by targetAngleCache*/

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