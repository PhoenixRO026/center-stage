package org.firstinspires.ftc.teamcode.tests

import com.qualcomm.robotcore.eventloop.opmode.Disabled
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.systems.Camera

@Disabled
@TeleOp(group = "Debug")
class AprilTagTest : LinearOpMode() {
    override fun runOpMode() {
        val camera = Camera(hardwareMap)

        camera.enableAprilTagDetection()
        camera.lowerExposure()

        while (opModeInInit()) {
            camera.update()
        }

        while (opModeIsActive()) {
            camera.update()

            camera.runDetection()?.let {
                telemetry.addData("x", it.x.inch)
                telemetry.addData("y", it.y.inch)
                telemetry.update()
            }
        }
    }
}