package org.firstinspires.ftc.teamcode.evenimente.beclean

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.canvas.Canvas
import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.acmerobotics.roadrunner.Pose2d
import com.acmerobotics.roadrunner.ftc.runBlocking
import com.phoenix_ro026.phoenixlib.units.rad
import com.phoenix_ro026.phoenixlib.units.rev
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.evenimente.beclean.robot.Robot2
import org.firstinspires.ftc.teamcode.evenimente.beclean.robot.systems.DetectionPipeline
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive

@TeleOp
class TenTurnTest : LinearOpMode() {
    @Config
    data object TurnTest {
        @JvmField var turn = 3600.0
    }
    override fun runOpMode() {
        val start = Pose2d(0.0, 0.0, 0.0)
        val drive = MecanumDrive(hardwareMap, start)

        val action = drive.actionBuilder(start)
            .turn(Math.toRadians(TurnTest.turn))
            .build()

        waitForStart()

        val dash = FtcDashboard.getInstance()
        val c = Canvas()
        action.preview(c)

        var running = true
        while (isStarted && !isStopRequested && running) {
            val p = TelemetryPacket()
            p.fieldOverlay().operations.addAll(c.operations)

            running = action.run(p)

            dash.sendTelemetryPacket(p)
        }

        /*//telemetry = MultipleTelemetry(telemetry, dash.telemetry)
        val startPose = Pose2d(0.0, 0.0, Math.toRadians(0.0))

        val robot = Robot2(hardwareMap, null, startPose, false, roadRunnerAuto = true)
        robot.camera.setColor(DetectionPipeline.DetectionColor.RED)
        val drive = robot.drive

        val action = drive.actionBuilder(startPose)
            .turn(10.rev)
            .build()

        waitForStart()

        val dash = FtcDashboard.getInstance()

        val c = Canvas()
        action.preview(c)
        var running = true

        while (isStarted && !isStopRequested && running) {
            robot.update()

            val p = TelemetryPacket()
            p.fieldOverlay().operations.addAll(c.operations)

            running = action.run(p)

            //telemetry.addData("heading", robot.drive.drive.pose.heading.log().rad.toDegrees())

            dash.sendTelemetryPacket(p)
            //telemetry.update()
        }*/
    }
}