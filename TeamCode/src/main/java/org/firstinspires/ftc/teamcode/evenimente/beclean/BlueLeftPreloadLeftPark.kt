package org.firstinspires.ftc.teamcode.evenimente.beclean

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.canvas.Canvas
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.acmerobotics.roadrunner.ParallelAction
import com.acmerobotics.roadrunner.SequentialAction
import com.phoenix_ro026.phoenixlib.units.Pose
import com.phoenix_ro026.phoenixlib.units.cm
import com.phoenix_ro026.phoenixlib.units.deg
import com.phoenix_ro026.phoenixlib.units.inch
import com.phoenix_ro026.phoenixlib.units.unaryMinus
import com.phoenix_ro026.phoenixlib.units.minus
import com.phoenix_ro026.phoenixlib.units.plus
import com.phoenix_ro026.phoenixlib.units.sec
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.teamcode.evenimente.beclean.robot.ARM_PIXEL_DROP_POSE
import org.firstinspires.ftc.teamcode.evenimente.beclean.robot.ARM_SCORE_POS
import org.firstinspires.ftc.teamcode.evenimente.beclean.robot.CLAW_PIXEL_DROP_ANGLE
import org.firstinspires.ftc.teamcode.evenimente.beclean.robot.CLAW_RAMP_ANGLE
import org.firstinspires.ftc.teamcode.evenimente.beclean.robot.CLAW_SCORE_ANGLE
import org.firstinspires.ftc.teamcode.evenimente.beclean.robot.LIFT_PASS_POSE
import org.firstinspires.ftc.teamcode.evenimente.beclean.robot.LIFT_PIXEL_DROP_POSE
import org.firstinspires.ftc.teamcode.evenimente.beclean.robot.Robot2
import org.firstinspires.ftc.teamcode.evenimente.beclean.robot.systems.DetectionPipeline

@Autonomous
class BlueLeftPreloadLeftPark : LinearOpMode() {

    override fun runOpMode() {
        val dash = FtcDashboard.getInstance()
        telemetry = MultipleTelemetry(telemetry, dash.telemetry)
        val startPose = Pose(12.inch, 61.inch, 90.deg)
        val leftPurple = Pose(26.inch, 45.inch, 90.deg)
        val leftYellow = Pose(46.inch, 40.inch + 4.cm, 180.deg)
        val middlePurple = Pose(20.inch - 4.cm, 38.inch, 90.deg)
        val middleYellow = Pose(46.inch, 35.inch - 1.cm, 180.deg)
        val rightPurple = Pose(15.inch, 32.inch, 0.deg)
        val rightYellow = Pose(46.inch, 35.inch, 180.deg)

        val robot = Robot2(hardwareMap, telemetry, startPose.toPose2d(), true, true)
        robot.camera.setColor(DetectionPipeline.DetectionColor.BLUE)
        robot.camera.openCamera()
        val drive = robot.drive

        val actionRight = SequentialAction(
            SequentialAction(
                robot.lift.goToPos(LIFT_PASS_POSE),
                ParallelAction(
                    robot.arm.goToPos(ARM_PIXEL_DROP_POSE),
                    robot.claw.goToAngle(CLAW_PIXEL_DROP_ANGLE)
                ),
                robot.lift.goToPos(LIFT_PIXEL_DROP_POSE)
            ),
            drive.actionBuilder(startPose.toPose2d())
                .strafeToLinearHeading(rightPurple.position, rightPurple.heading)
                .build(),
            robot.claw.openLeftClaw(),
            robot.lift.goToPos(LIFT_PASS_POSE),
            robot.claw.closeClaw(),
            robot.arm.goToPos(0.0),
            drive.actionBuilder(rightPurple.toPose2d())
                .strafeToLinearHeading(rightYellow.position, rightYellow.heading)
                .build(),
            ParallelAction(
                robot.arm.goToPos(ARM_SCORE_POS),
                robot.claw.goToAngle(CLAW_SCORE_ANGLE)
            ),
            drive.actionBuilder(rightYellow.toPose2d())
                .setTangent(0.deg)
                .lineToX(rightYellow.position.x + 2.inch + 1.cm)
                .wait(0.5.sec)
                .stopAndAdd(robot.claw.openRightClaw())
                .setTangent(180.deg)
                .lineToX(rightYellow.position.x - 2.inch - 1.cm)
                .stopAndAdd(SequentialAction(
                    robot.claw.closeClaw(),
                    ParallelAction(
                        robot.arm.goToPos(0.0),
                        robot.claw.goToAngle(CLAW_RAMP_ANGLE)
                    ),
                    robot.lift.goToPos(0)
                ))
                .setTangent(-90.deg)
                .lineToY(58.inch)
                .setTangent(0.deg)
                .lineToX(rightYellow.position.x + 2.inch)
                .build()
        )

        val actionMiddle = SequentialAction(
            SequentialAction(
                robot.lift.goToPos(LIFT_PASS_POSE),
                ParallelAction(
                    robot.arm.goToPos(ARM_PIXEL_DROP_POSE),
                    robot.claw.goToAngle(CLAW_PIXEL_DROP_ANGLE)
                ),
                robot.lift.goToPos(LIFT_PIXEL_DROP_POSE)
            ),
            drive.actionBuilder(startPose.toPose2d())
                .strafeToLinearHeading(middlePurple.position, middlePurple.heading)
                .build(),
            robot.claw.openLeftClaw(),
            robot.lift.goToPos(LIFT_PASS_POSE),
            robot.claw.closeClaw(),
            robot.arm.goToPos(0.0),
            drive.actionBuilder(middlePurple.toPose2d())
                .strafeToLinearHeading(middleYellow.position, middleYellow.heading)
                .build(),
            ParallelAction(
                robot.arm.goToPos(ARM_SCORE_POS),
                robot.claw.goToAngle(CLAW_SCORE_ANGLE)
            ),
            drive.actionBuilder(middleYellow.toPose2d())
                .setTangent(0.deg)
                .lineToX(middleYellow.position.x + 2.inch + 1.cm)
                .wait(0.5.sec)
                .stopAndAdd(robot.claw.openRightClaw())
                .setTangent(180.deg)
                .lineToX(middleYellow.position.x - 2.inch - 1.cm)
                .stopAndAdd(SequentialAction(
                    robot.claw.closeClaw(),
                    ParallelAction(
                        robot.arm.goToPos(0.0),
                        robot.claw.goToAngle(CLAW_RAMP_ANGLE)
                    ),
                    robot.lift.goToPos(0)
                ))
                .setTangent(-90.deg)
                .lineToY(58.inch)
                .setTangent(0.deg)
                .lineToX(middleYellow.position.x + 2.inch)
                .build()
        )

        val actionLeft = SequentialAction(
            SequentialAction(
                robot.lift.goToPos(LIFT_PASS_POSE),
                ParallelAction(
                    robot.arm.goToPos(ARM_PIXEL_DROP_POSE),
                    robot.claw.goToAngle(CLAW_PIXEL_DROP_ANGLE)
                ),
                robot.lift.goToPos(LIFT_PIXEL_DROP_POSE)
            ),
            drive.actionBuilder(startPose.toPose2d())
                .strafeToLinearHeading(leftPurple.position, leftPurple.heading)
                .build(),
            robot.claw.openLeftClaw(),
            robot.lift.goToPos(LIFT_PASS_POSE),
            robot.claw.closeClaw(),
            robot.arm.goToPos(0.0),
            drive.actionBuilder(leftPurple.toPose2d())
                .strafeToLinearHeading(leftYellow.position, leftYellow.heading)
                .build(),
            ParallelAction(
                robot.arm.goToPos(ARM_SCORE_POS),
                robot.claw.goToAngle(CLAW_SCORE_ANGLE)
            ),
            drive.actionBuilder(leftYellow.toPose2d())
                .setTangent(0.deg)
                .lineToX(leftYellow.position.x + 2.inch + 1.cm)
                .wait(0.5.sec)
                .stopAndAdd(robot.claw.openRightClaw())
                .setTangent(180.deg)
                .lineToX(leftYellow.position.x - 2.inch - 1.cm)
                .stopAndAdd(SequentialAction(
                    robot.claw.closeClaw(),
                    ParallelAction(
                        robot.arm.goToPos(0.0),
                        robot.claw.goToAngle(CLAW_RAMP_ANGLE)
                    ),
                    robot.lift.goToPos(0)
                ))
                .setTangent(-90.deg)
                .lineToY(58.inch)
                .setTangent(0.deg)
                .lineToX(rightYellow.position.x + 2.inch)
                .build()
        )

        while (opModeInInit()) {
            robot.camera.displayDetection()
            telemetry.update()
            sleep(10)
        }
        val action = when(robot.camera.detectionPosition) {
            DetectionPipeline.DetectionPosition.LEFT -> actionLeft
            DetectionPipeline.DetectionPosition.CENTER -> actionMiddle
            DetectionPipeline.DetectionPosition.RIGHT -> actionRight
        }

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