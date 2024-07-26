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
import org.firstinspires.ftc.teamcode.stc.robot.Arm
import org.firstinspires.ftc.teamcode.stc.robot.Claw
import org.firstinspires.ftc.teamcode.stc.robot.Lift

@Autonomous
class RedRight: LinearOpMode() {
    override fun runOpMode() {

        val dash = FtcDashboard.getInstance()

        telemetry = MultipleTelemetry(telemetry, dash.telemetry)

        val startPose = Pose2d(12.0, -62.0, Math.toRadians(-90.0))
        val midPurple = Pose2d(12.0, -36.0, Math.toRadians(-90.0))
        val midBoard = Pose2d(48.0, -36.0, Math.toRadians(180.0))

        val lift = Lift(hardwareMap)
        val claw = Claw(hardwareMap)
        val arm = Arm(hardwareMap)
        val drive = MecanumDrive(hardwareMap, startPose)

        val action = drive.actionBuilder(startPose)
            .splineTo(midPurple.position, Math.toRadians(90.0))
            .setTangent(Math.toRadians(-90.0))
            .lineToY(-48.0)
            .afterTime(0.0, SequentialAction(
                lift.goToPass(),
                ParallelAction(
                    arm.goToScore(),
                    claw.tiltToScore()
                ),
                lift.goToYellow()
            ))
            .setTangent(Math.toRadians(90.0))
            .splineTo(midBoard.position, Math.toRadians(0.0))
            .stopAndAdd(SequentialAction(
                claw.openClaw(),
                SleepAction(0.5.s),
                claw.closeRight()
            ))
            .setTangent(Math.toRadians(180.0))
            .lineToX(40.0)
            .afterTime(0.0, SequentialAction(
                lift.goToPass(),
                ParallelAction(
                    arm.goToIntake(),
                    claw.tiltToIntake()
                ),
                lift.goToIntake()
            ))
            .setTangent(Math.toRadians(-90.0))
            .lineToY(-60.0)
            .build()


        val canvas = Canvas()

        waitForStart()

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