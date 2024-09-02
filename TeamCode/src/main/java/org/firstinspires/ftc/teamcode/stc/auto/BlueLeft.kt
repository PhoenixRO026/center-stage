package org.firstinspires.ftc.teamcode.stc.auto

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.canvas.Canvas
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.acmerobotics.roadrunner.ParallelAction
import com.acmerobotics.roadrunner.Pose2d
import com.acmerobotics.roadrunner.SequentialAction
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.teamcode.lib.units.SleepAction
import org.firstinspires.ftc.teamcode.lib.units.s
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive
import org.firstinspires.ftc.teamcode.robot.ColorVisionProcessor
import org.firstinspires.ftc.teamcode.stc.robot.Arm
import org.firstinspires.ftc.teamcode.stc.robot.Camera
import org.firstinspires.ftc.teamcode.stc.robot.Claw
import org.firstinspires.ftc.teamcode.stc.robot.Lift
@Autonomous
class BlueLeft: LinearOpMode() {
    override fun runOpMode() {

        val dash = FtcDashboard.getInstance()

        telemetry = MultipleTelemetry(telemetry, dash.telemetry)

        val startPose = Pose2d(12.0, 62.0, Math.toRadians(90.0))
        val midPurple = Pose2d(12.0, 36.0, Math.toRadians(90.0))
        val leftPurple = Pose2d(17.0, 37.0, Math.toRadians(-45.0))
        val rightPurple = Pose2d(9.0, 37.0, Math.toRadians(-90.0))
        val turnPoint = Pose2d(15.0, 55.0, Math.toRadians(90.0))
        val midBoard = Pose2d(48.0, 36.0, Math.toRadians(0.0))
        val leftBoard = Pose2d(48.0, 42.0, Math.toRadians(180.0))
        val rightBoard = Pose2d(48.0, 30.0, Math.toRadians(180.0))

        /*val lift = Lift(hardwareMap)
        val claw = Claw(hardwareMap)
        val arm = Arm(hardwareMap)*/
        val drive = MecanumDrive(hardwareMap, startPose)
        val camera = Camera(hardwareMap, telemetry)
        camera.setColor(ColorVisionProcessor.DetectionColor.RED)

        val actionMiddle = drive.actionBuilder(startPose)
            .setTangent(Math.toRadians(90.0))
            .lineToY(30.0)
            .lineToY(55.0)
            .setTangent(Math.toRadians(-90.0))
            .splineTo(midBoard.position, Math.toRadians(0.0))
            .waitSeconds(1.0)
            .setTangent(Math.toRadians(180.0))
            .lineToX(40.0)
            .setTangent(Math.toRadians(-90.0))
            .lineToY(60.0)
            .build()

        val actionLeft = drive.actionBuilder(startPose)
            .setTangent(Math.toRadians(-90.0))
            .splineTo(leftPurple.position, Math.toRadians(-45.0))
            .setTangent(Math.toRadians(135.0))
            .splineTo(turnPoint.position, Math.toRadians(90.0))
            .setTangent(Math.toRadians(-90.0))
            .splineTo(leftBoard.position, Math.toRadians(0.0))
            .waitSeconds(1.0)
            .setTangent(Math.toRadians(180.0))
            .lineToX(40.0)
            .setTangent(Math.toRadians(-90.0))
            .lineToY(60.0)
            .build()

        val actionRight = drive.actionBuilder(startPose)
            .setTangent(Math.toRadians(-90.0))
            .splineTo(rightPurple.position, Math.toRadians(-135.0))
            .setTangent(Math.toRadians(45.0))
            .splineTo(turnPoint.position, Math.toRadians(90.0))
            .setTangent(Math.toRadians(-90.0))
            .splineTo(rightBoard.position, Math.toRadians(0.0))
            .waitSeconds(1.0)
            .setTangent(Math.toRadians(180.0))
            .lineToX(40.0)
            .setTangent(Math.toRadians(-90.0))
            .lineToY(60.0)
            .build()


        val canvas = Canvas()

        while (opModeInInit()) {
            camera.displayDetection()
            sleep(10)
        }

        val action = when(camera.detectionPosition) {
            ColorVisionProcessor.DetectionPosition.LEFT -> actionLeft
            ColorVisionProcessor.DetectionPosition.CENTER -> actionMiddle
            ColorVisionProcessor.DetectionPosition.RIGHT -> actionRight
        }

        action.preview(canvas)

        var running = true

        while (running && opModeIsActive()) {
            val packet = TelemetryPacket()
            packet.fieldOverlay().operations.addAll(canvas.operations)

            running = action.run(packet)

            dash.sendTelemetryPacket(packet)
        }
    }
}