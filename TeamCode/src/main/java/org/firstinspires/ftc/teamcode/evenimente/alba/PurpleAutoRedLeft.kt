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
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.teamcode.evenimente.alba.robot.ARM_PIXEL_DROP_POSE
import org.firstinspires.ftc.teamcode.evenimente.alba.robot.CLAW_PIXEL_DROP_POSE
import org.firstinspires.ftc.teamcode.evenimente.alba.robot.LIFT_PASS_POSE
import org.firstinspires.ftc.teamcode.evenimente.alba.robot.LIFT_PIXEL_DROP_POSE
import org.firstinspires.ftc.teamcode.evenimente.alba.robot.Robot2
import org.firstinspires.ftc.teamcode.evenimente.liga_wonder.robot.ARM_RAMP_POS
import org.firstinspires.ftc.teamcode.evenimente.liga_wonder.robot.CLAW_RAMP_POS
import org.firstinspires.ftc.teamcode.evenimente.liga_wonder.robot.systems.DetectionPipeline
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive

@Autonomous
class PurpleAutoRedLeft : LinearOpMode() {
    override fun runOpMode() {
        val dash = FtcDashboard.getInstance()
        telemetry = MultipleTelemetry(telemetry, dash.telemetry)
        val startPose = Pose2d(-36.0, 61.0, Math.toRadians(90.0))
        //val purplePixelPoseLeft = Pose2d(-32.0, 44.0, Math.toRadians(135.0))
        //val purplePixelPoseRight = Pose2d(-30.0 - 6, 44.0, Math.toRadians(45.0))
        val purplePixelPoseLeft = Pose2d(-34.0, 44.0, Math.toRadians(135.0))
        val purplePixelPoseRight = Pose2d(-30.0 - 6 + 2, 39.0, Math.toRadians(20.0))
        val purplePixelPoseMidle = Pose2d(-29.0,40.0, Math.toRadians(90.0))
        val stackPose = Pose2d(-58.0, 11.0, 0.0)
        val afterTrussPose = Pose2d(20.0, 11.0, Math.toRadians(180.0))
        val boardPose = Pose2d(48.0, 34.0, Math.toRadians(180.0))

        val robot = Robot2(hardwareMap, telemetry, startPose, false, roadRunnerAuto = true)
        robot.camera.setColor(DetectionPipeline.DetectionColor.RED)
        val drive = robot.drive.drive

        val slowSpeed = MinVelConstraint(listOf(
            drive.kinematics.WheelVelConstraint(10.0),
            AngularVelConstraint(MecanumDrive.PARAMS.maxAngVel)
        ))

        robot.camera.openCamera()

        val actionLEFT = SequentialAction(
            ParallelAction(
                drive.actionBuilder(startPose)
                    .setTangent(Math.toRadians(-90.0))
                    .splineToLinearHeading(purplePixelPoseLeft, Math.toRadians(-90.0))
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
            robot.claw.openLeftClaw(),
            robot.claw.closeClaw(),
            robot.lift.goToPos(LIFT_PASS_POSE),
            ParallelAction(
                robot.arm.goToPos(0.0),
                robot.claw.goToAngle(CLAW_RAMP_POS)
            ),
            robot.lift.goToPos(0),
            /*drive.actionBuilder(purplePixelPoseLeft)
                .turn(Math.toRadians(-160.0))
                .setTangent(Math.toRadians(90.0))
                .lineToY(60.0)
                .setTangent(Math.toRadians(180.0))
                .lineToX(purplePixelPoseRight.position.x - 36)
                .build(),
            robot.lift.goToPos(LIFT_PASS_POSE),
            ParallelAction(
                robot.arm.goToPos(ARM_PIXEL_DROP_POSE),
                robot.claw.goToAngle(CLAW_PIXEL_DROP_POSE)
            ),
            robot.claw.openRightClaw(),
            robot.claw.closeClaw(),
            ParallelAction(
                robot.arm.goToPos(0.0),
                robot.claw.goToAngle(CLAW_RAMP_POS)
            ),
            robot.lift.goToPos(0)*/
        )

        val actionRIGHT = SequentialAction(
            ParallelAction(
                drive.actionBuilder(startPose)
                    .setTangent(Math.toRadians(-90.0))
                    .splineToLinearHeading(purplePixelPoseRight, Math.toRadians(-90.0))
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
            robot.claw.openLeftClaw(),
            robot.claw.closeClaw(),
            robot.lift.goToPos(LIFT_PASS_POSE),
            ParallelAction(
                robot.arm.goToPos(0.0),
                robot.claw.goToAngle(CLAW_RAMP_POS)
            ),
            robot.lift.goToPos(0),
            /*drive.actionBuilder(purplePixelPoseRight)
                .turn(Math.toRadians(-45.0))
                .setTangent(Math.toRadians(90.0))
                .lineToY(60.0)
                .setTangent(Math.toRadians(180.0))
                .lineToX(purplePixelPoseRight.position.x - 36)
                .build(),
            robot.lift.goToPos(LIFT_PASS_POSE),
            ParallelAction(
                robot.arm.goToPos(ARM_PIXEL_DROP_POSE),
                robot.claw.goToAngle(CLAW_PIXEL_DROP_POSE)
            ),
            robot.claw.openRightClaw(),
            robot.claw.closeClaw(),
            ParallelAction(
                robot.arm.goToPos(0.0),
                robot.claw.goToAngle(CLAW_RAMP_POS)
            ),
            robot.lift.goToPos(0)*/
        )

        val actionMIDDLE = SequentialAction(
            ParallelAction(
                drive.actionBuilder(startPose)
                    .setTangent(Math.toRadians(-90.0))
                    .splineToLinearHeading(purplePixelPoseMidle, Math.toRadians(-90.0))
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
            robot.claw.openLeftClaw(),
            robot.claw.closeClaw(),
            robot.lift.goToPos(LIFT_PASS_POSE),
            ParallelAction(
                robot.arm.goToPos(0.0),
                robot.claw.goToAngle(CLAW_RAMP_POS)
            ),
            robot.lift.goToPos(0),
            /*drive.actionBuilder(purplePixelPoseMidle)
                .turn(Math.toRadians(-90.0))
                .setTangent(Math.toRadians(90.0))
                .lineToY(60.0)
                .setTangent(Math.toRadians(180.0))
                .lineToX(purplePixelPoseRight.position.x - 36)
                .build(),
            robot.claw.openRightClaw(),
            robot.claw.closeClaw(),
            ParallelAction(
                robot.arm.goToPos(0.0),
                robot.claw.goToAngle(CLAW_RAMP_POS)
            ),
            robot.lift.goToPos(0)*/
        )

        while (opModeInInit()) {
            robot.camera.displayDetection()
            telemetry.update()
            sleep(10)
        }
        val action = when(robot.camera.detectionPosition) {
            DetectionPipeline.DetectionPosition.LEFT -> actionLEFT
            DetectionPipeline.DetectionPosition.CENTER -> actionMIDDLE
            DetectionPipeline.DetectionPosition.RIGHT -> actionRIGHT
        }
        //robot.camera.stopStream()

        val c = Canvas()
        action.preview(c)
        var running = true

        while (isStarted && !isStopRequested && running) {
            robot.update()

            val p = TelemetryPacket()
            p.fieldOverlay().operations.addAll(c.operations)

            running = action.run(p)

            dash.sendTelemetryPacket(p)
            telemetry.update()
        }
    }
}