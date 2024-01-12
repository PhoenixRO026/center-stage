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


@Autonomous
class CameraTest : LinearOpMode() {
    override fun runOpMode() {
        val dash = FtcDashboard.getInstance()
        telemetry = MultipleTelemetry(telemetry, dash.telemetry)
        val startPose = Pose2d(12.0, 61.0, Math.toRadians(90.0))

        val robot = Robot(hardwareMap, telemetry, startPose, false)
        robot.camera.openCamera()

        val action = robot.drive.drive.actionBuilder(startPose)
            .lineToY(61.0 - 5)
            .strafeTo(Vector2d(12.0 + 48 , 61.0 - 5))
            .build()

        while (opModeInInit()) {
            robot.camera.displayDetection()
            telemetry.update()
            sleep(50)
        }
        robot.camera.stopStream()

        val c = Canvas()
        action.preview(c)
        var running = true

        while (isStarted && !isStopRequested && running) {
            robot.update()

            val p = TelemetryPacket()
            p.fieldOverlay().operations.addAll(c.operations)

            running = action.run(p)

            dash.sendTelemetryPacket(p)
        }
    }
}