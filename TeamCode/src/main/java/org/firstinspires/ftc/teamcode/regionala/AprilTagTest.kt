package org.firstinspires.ftc.teamcode.regionala

import com.acmerobotics.dashboard.FtcDashboard
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.teamcode.robot.Camera

@Autonomous
class AprilTagTest : LinearOpMode() {
    override fun runOpMode() {
        telemetry = FtcDashboard.getInstance().telemetry

        val camera = Camera(hardwareMap, telemetry)
        camera.disableColorDetection()
        camera.enableAprilTagDetection()

        waitForStart()

        while (opModeIsActive()) {
            sleep(20)
        }
    }
}