package org.firstinspires.ftc.teamcode.evenimente.liga_wonder.robot

import com.acmerobotics.roadrunner.Pose2d
import com.qualcomm.hardware.lynx.LynxModule
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.evenimente.liga_wonder.robot.systems.Arm
import org.firstinspires.ftc.teamcode.evenimente.liga_wonder.robot.systems.Claw
import org.firstinspires.ftc.teamcode.evenimente.liga_wonder.robot.systems.Drive
import org.firstinspires.ftc.teamcode.evenimente.liga_wonder.robot.systems.Intake
import org.firstinspires.ftc.teamcode.evenimente.liga_wonder.robot.systems.Lift

class Robot(
    hardwareMap: HardwareMap,
    private val telemetry: Telemetry? = null,
    startPose: Pose2d = Pose2d(0.0, 0.0, 0.0)
) {
    private val hubs = hardwareMap.getAll(LynxModule::class.java)
    val arm = Arm(hardwareMap, telemetry, ARM_RAMP_POS..1.0)
    //val camera = Camera(hardwareMap, telemetry)
    val lift = Lift(hardwareMap, telemetry)
    val drive = Drive(hardwareMap, telemetry, startPose)
    val intake = Intake(hardwareMap, telemetry)
    val claw = Claw(hardwareMap, telemetry, FINGERS_RANGE)

    var deltaTime = 1.0
    private var oldTime = System.nanoTime() / 1E9

    init {
        hubs.forEach {
            it.bulkCachingMode = LynxModule.BulkCachingMode.MANUAL
        }
    }

    fun update() {
        val currentTime = System.nanoTime() / 1E9
        deltaTime = (currentTime - oldTime) * 60.0
        oldTime = currentTime
        telemetry?.addData("delta time", deltaTime)
        hubs.forEach {
            it.clearBulkCache()
        }
        drive.update()
        lift.update()
    }
}