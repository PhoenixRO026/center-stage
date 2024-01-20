package org.firstinspires.ftc.teamcode.evenimente.beclean.robot

import com.acmerobotics.roadrunner.Pose2d
import com.qualcomm.hardware.lynx.LynxModule
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.evenimente.beclean.robot.systems.Arm2
import org.firstinspires.ftc.teamcode.evenimente.beclean.robot.systems.Camera
import org.firstinspires.ftc.teamcode.evenimente.beclean.robot.systems.Claw2
import org.firstinspires.ftc.teamcode.evenimente.beclean.robot.systems.Drive2
import org.firstinspires.ftc.teamcode.evenimente.beclean.robot.systems.Intake2
import org.firstinspires.ftc.teamcode.evenimente.beclean.robot.systems.Lift2
import org.firstinspires.ftc.teamcode.evenimente.beclean.robot.systems.Lift3
import org.firstinspires.ftc.teamcode.evenimente.beclean.robot.systems.Plane

class Robot3(
    hardwareMap: HardwareMap,
    private val telemetry: Telemetry? = null,
    startPose: Pose2d = Pose2d(0.0, 0.0, 0.0),
    private val manualCache: Boolean = true,
    private val roadRunnerAuto: Boolean = false
) {
    private val hubs = hardwareMap.getAll(LynxModule::class.java)
    @JvmField
    val arm = Arm2(hardwareMap, telemetry, ARM_RAMP_POS..1.0)
    @JvmField
    val camera = Camera(hardwareMap, telemetry)
    @JvmField
    val lift = Lift3(hardwareMap, telemetry)
    @JvmField
    val drive = Drive2(hardwareMap, telemetry, startPose)
    @JvmField
    val intake = Intake2(hardwareMap, telemetry)
    @JvmField
    val claw = Claw2(hardwareMap, telemetry, FINGERS_RANGE, CLAW_RANGE)
    @JvmField
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
                it.getBulkData()
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