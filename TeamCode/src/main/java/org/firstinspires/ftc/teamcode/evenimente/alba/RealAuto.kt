package org.firstinspires.ftc.teamcode.evenimente.alba

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.canvas.Canvas
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.acmerobotics.roadrunner.AngularVelConstraint
import com.acmerobotics.roadrunner.MinVelConstraint
import com.acmerobotics.roadrunner.ParallelAction
import com.acmerobotics.roadrunner.Pose2d
import com.acmerobotics.roadrunner.SequentialAction
import com.acmerobotics.roadrunner.Vector2d
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.teamcode.evenimente.alba.robot.ARM_PIXEL_DROP_POSE
import org.firstinspires.ftc.teamcode.evenimente.alba.robot.ARM_STACK_POSE
import org.firstinspires.ftc.teamcode.evenimente.alba.robot.CLAW_PIXEL_DROP_POSE
import org.firstinspires.ftc.teamcode.evenimente.alba.robot.CLAW_STACK_POSE
import org.firstinspires.ftc.teamcode.evenimente.alba.robot.LIFT_PASS_POSE
import org.firstinspires.ftc.teamcode.evenimente.alba.robot.LIFT_PIXEL_DROP_POSE
import org.firstinspires.ftc.teamcode.evenimente.alba.robot.LIFT_STACK_POSE
import org.firstinspires.ftc.teamcode.evenimente.alba.robot.Robot2
import org.firstinspires.ftc.teamcode.evenimente.liga_wonder.robot.ARM_RAMP_POS
import org.firstinspires.ftc.teamcode.evenimente.liga_wonder.robot.CLAW_RAMP_POS
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive

//@Autonomous
class RealAuto : LinearOpMode() {
    override fun runOpMode() {
        val dash = FtcDashboard.getInstance()
        telemetry = MultipleTelemetry(telemetry, dash.telemetry)
        val startPose = Pose2d(-36.0, 61.0, Math.toRadians(90.0))
        val purplePixelPose = Pose2d(-32.0, 36.0, Math.toRadians(135.0))
        val stackPose = Pose2d(-58.0, 11.0, 0.0)
        val afterTrussPose = Pose2d(20.0, 11.0, Math.toRadians(180.0))
        val boardPose = Pose2d(48.0, 34.0, Math.toRadians(180.0))

        val robot = Robot2(hardwareMap, telemetry, startPose, false, roadRunnerAuto = true)
        val drive = robot.drive.drive

        val slowSpeed = MinVelConstraint(listOf(
            drive.kinematics.WheelVelConstraint(10.0),
            AngularVelConstraint(MecanumDrive.PARAMS.maxAngVel)
        ))

        robot.camera.openCamera()

        val action = SequentialAction(
            ParallelAction(
                drive.actionBuilder(startPose)
                    .setTangent(Math.toRadians(-90.0))
                    .splineToLinearHeading(purplePixelPose, Math.toRadians(-90.0))
                    .build(),
                SequentialAction(
                    robot.lift.goToPos(LIFT_PASS_POSE),
                    ParallelAction(
                        robot.claw.goToAngle(CLAW_PIXEL_DROP_POSE),
                        robot.arm.goToPos(ARM_PIXEL_DROP_POSE)
                    ),
                    robot.lift.goToPos(LIFT_PIXEL_DROP_POSE)
                )
            ),
            robot.claw.openClaw(),
            ParallelAction(
                robot.claw.goToAngle(CLAW_STACK_POSE),
                robot.arm.goToPos(ARM_STACK_POSE),
                robot.lift.goToPos(LIFT_STACK_POSE)
            ),
            drive.actionBuilder(purplePixelPose)
                .turn(Math.toRadians(-45.0))
                .setTangent(Math.toRadians(-90.0))
                .splineTo(Vector2d(stackPose.position.x + 4, stackPose.position.y), Math.toRadians(180.0))
                .splineTo(stackPose.position, Math.toRadians(180.0), slowSpeed)
                .build(),
            robot.claw.closeClaw(),

            ParallelAction(
                drive.actionBuilder(stackPose)
                    .setTangent(0.0)
                    .splineTo(Vector2d(stackPose.position.x + 8, stackPose.position.y), 0.0)
                    .stopAndAdd(SequentialAction(
                        robot.lift.goToPos(LIFT_PASS_POSE),
                        ParallelAction(
                            robot.claw.goToAngle(CLAW_RAMP_POS),
                            robot.arm.goToPos(ARM_RAMP_POS)
                        ),
                        robot.lift.goToPos(0)
                    ))
                    .splineTo(Vector2d(afterTrussPose.position.x - 8, afterTrussPose.position.y), 0.0)
                    .splineToSplineHeading(afterTrussPose, 0.0)
                    //.splineTo()
                    .build()

            )
        )

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