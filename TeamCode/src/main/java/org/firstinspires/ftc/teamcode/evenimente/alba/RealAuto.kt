package org.firstinspires.ftc.teamcode.evenimente.alba

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.acmerobotics.roadrunner.Pose2d
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.teamcode.evenimente.alba.robot.Robot2

@Autonomous
class RealAuto : LinearOpMode() {
    override fun runOpMode() {
        telemetry = MultipleTelemetry(telemetry, FtcDashboard.getInstance().telemetry)
        val startPose = Pose2d(12.0, 61.0, Math.toRadians(90.0))

        val robot = Robot2(hardwareMap, telemetry, startPose, true, true)
        robot.camera.openCamera()

        while (opModeInInit()) {
            robot.update()
            robot.camera.displayDetection()

        }
    }
}