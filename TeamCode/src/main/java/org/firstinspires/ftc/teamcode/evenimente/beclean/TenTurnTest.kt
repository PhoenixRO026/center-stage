package org.firstinspires.ftc.teamcode.evenimente.beclean

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.canvas.Canvas
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.acmerobotics.roadrunner.Pose2d
import com.phoenix_ro026.phoenixlib.units.rad
import com.phoenix_ro026.phoenixlib.units.rev
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.teamcode.evenimente.beclean.robot.Robot2
import org.firstinspires.ftc.teamcode.evenimente.beclean.robot.systems.DetectionPipeline

@Autonomous
class TenTurnTest : LinearOpMode() {
    override fun runOpMode() {
        val dash = FtcDashboard.getInstance()
        telemetry = MultipleTelemetry(telemetry, dash.telemetry)
        val startPose = Pose2d(0.0, 0.0, Math.toRadians(0.0))

        val robot = Robot2(hardwareMap, telemetry, startPose, false, roadRunnerAuto = true)
        robot.camera.setColor(DetectionPipeline.DetectionColor.RED)
        val drive = robot.drive

        val action = drive.actionBuilder(startPose)
            .turn(10.rev)
            .build()

        waitForStart()

        val c = Canvas()
        action.preview(c)
        var running = true

        while (isStarted && !isStopRequested && running) {
            robot.update()

            val p = TelemetryPacket()
            p.fieldOverlay().operations.addAll(c.operations)

            running = action.run(p)

            telemetry.addData("heading", robot.drive.drive.pose.heading.log().rad.toDegrees())

            dash.sendTelemetryPacket(p)
            telemetry.update()
        }
    }
}