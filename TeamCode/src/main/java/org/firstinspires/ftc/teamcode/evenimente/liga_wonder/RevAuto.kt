package org.firstinspires.ftc.teamcode.evenimente.liga_wonder

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.canvas.Canvas
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.acmerobotics.roadrunner.Pose2d
import com.acmerobotics.roadrunner.Vector2d
import com.acmerobotics.roadrunner.ftc.runBlocking
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.teamcode.evenimente.liga_wonder.robot.Robot
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive


@Autonomous
class RevAuto : LinearOpMode() {
    override fun runOpMode() {
        telemetry = MultipleTelemetry(telemetry, FtcDashboard.getInstance().telemetry)
        val startPose = Pose2d(12.0, 61.0, Math.toRadians(90.0))

        //val robot = Robot(hardwareMap, telemetry, startPose, false)
        //robot.camera.openCamera()
        val drive = MecanumDrive(hardwareMap, startPose)

        val action = drive.actionBuilder(startPose)
            .lineToY(61.0 - 5)
            .strafeTo(Vector2d(12.0 + 48 , 61.0 - 5))
            .build()

        while (opModeInInit()) {
           // robot.camera.displayDetection()
            telemetry.update()
            sleep(50)
        }

        runBlocking(action)
    }
}