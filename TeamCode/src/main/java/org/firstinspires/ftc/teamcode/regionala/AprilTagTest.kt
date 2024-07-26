package org.firstinspires.ftc.teamcode.regionala

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.Disabled
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.teamcode.robot.Camera

@Disabled
@Autonomous
class AprilTagTest : LinearOpMode() {
    override fun runOpMode() {
        telemetry = MultipleTelemetry(telemetry, FtcDashboard.getInstance().telemetry)

        val camera = Camera(hardwareMap, telemetry)
        camera.disableColorDetection()
        camera.enableAprilTagDetection()

        while (opModeInInit()) {
            /*telemetry.addData("tag 5 x inch", camera.aprilTag5Pose.position.x.inch)
            telemetry.addData("tag 5 y inch", camera.aprilTag5Pose.position.y.inch)
            telemetry.addData("tag 5 heading deg", camera.aprilTag5Pose.heading.deg)
            telemetry.update()*/
            if (camera.findTag5()) {
                telemetry.addData("tag 5 x inch", camera.robotPose.position.x.inch)
                telemetry.addData("tag 5 y inch", camera.robotPose.position.y.inch)
                telemetry.addData("tag 5 heading deg", camera.robotPose.heading.deg)
                telemetry.update()
            }
            /*if (camera.printTag5()) {
                telemetry.update()
            }*/

            sleep(20)
        }

        while (opModeIsActive()) {
            sleep(20)
        }
    }
}