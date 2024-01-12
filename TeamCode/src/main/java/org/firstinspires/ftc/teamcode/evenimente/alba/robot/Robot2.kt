package org.firstinspires.ftc.teamcode.evenimente.alba.robot

import com.acmerobotics.roadrunner.Pose2d
import com.qualcomm.hardware.lynx.LynxModule
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.evenimente.liga_wonder.robot.ARM_RAMP_POS
import org.firstinspires.ftc.teamcode.evenimente.liga_wonder.robot.FINGERS_RANGE
import org.firstinspires.ftc.teamcode.evenimente.liga_wonder.robot.systems.Arm
import org.firstinspires.ftc.teamcode.evenimente.liga_wonder.robot.systems.Camera
import org.firstinspires.ftc.teamcode.evenimente.liga_wonder.robot.systems.Claw
import org.firstinspires.ftc.teamcode.evenimente.liga_wonder.robot.systems.Drive
import org.firstinspires.ftc.teamcode.evenimente.liga_wonder.robot.systems.Intake
import org.firstinspires.ftc.teamcode.evenimente.liga_wonder.robot.systems.Lift
import org.firstinspires.ftc.teamcode.evenimente.liga_wonder.robot.systems.Plane

class Robot2(
    hardwareMap: HardwareMap,
    private val telemetry: Telemetry? = null,
    startPose: Pose2d = Pose2d(0.0, 0.0, 0.0),
    private val manualCache: Boolean = true,
    private val roadRunnerAuto: Boolean = false
) {
    private val hubs = hardwareMap.getAll(LynxModule::class.java)
    val arm = Arm2(hardwareMap, telemetry, ARM_RAMP_POS..1.0)
    val camera = Camera(hardwareMap, telemetry)
    val lift = Lift2(hardwareMap, telemetry)
    val drive = Drive(hardwareMap, telemetry, startPose)
    val intake = Intake(hardwareMap, telemetry)
    val claw = Claw2(hardwareMap, telemetry, FINGERS_RANGE)
    val plane = Plane(hardwareMap, telemetry)

    var deltaTime = 1.0
    private var oldTime = System.nanoTime() / 1E9

    init {
        if (manualCache) {
            hubs.forEach {
                it.bulkCachingMode = LynxModule.BulkCachingMode.MANUAL
            }
        } else {
            hubs.forEach {
                it.bulkCachingMode = LynxModule.BulkCachingMode.AUTO
            }
        }
    }

    fun update() {
        val currentTime = System.nanoTime() / 1E9
        deltaTime = (currentTime - oldTime) * 60.0
        oldTime = currentTime
        telemetry?.addData("delta time", deltaTime)
        if (manualCache) {
            hubs.forEach {
                it.clearBulkCache()
            }
        }
        if (!roadRunnerAuto) {
            drive.update()
        }
        lift.update()
        arm.update(deltaTime)
        claw.update(deltaTime)
    }
}