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
import org.firstinspires.ftc.teamcode.evenimente.beclean.robot.LIFT_SCORE_POSE
import org.firstinspires.ftc.teamcode.evenimente.beclean.robot.Robot2
import org.firstinspires.ftc.teamcode.evenimente.beclean.robot.systems.DetectionPipeline

@Autonomous
class RedLeftPreload : LinearOpMode() {

    override fun runOpMode() {
        val dash = FtcDashboard.getInstance()
        telemetry = MultipleTelemetry(telemetry, dash.telemetry)
        val startPose = Pose(-35.inch, -61.inch, -90.deg)
        val leftPurple = Pose(-35.inch - 8.cm, -38.inch - 18.cm, -45.deg)
        val leftLeavePos = Pose(-31.inch, -12.inch, 0.deg)
        val leftYellow = Pose(45.inch + 7.cm, -35.inch - 6.cm, 180.deg)
        val middlePurple = Pose(-50.inch, -22.inch, -180.deg)
        val middleLeavePos = Pose(-31.inch, -10.inch, 5.deg)
        val middleYellow = Pose(45.inch + 7.cm, -30.inch - 2.cm, 180.deg)
        val rightPurple = Pose(-36.inch, -35.inch, -180.deg)
        val rightLeavePos = Pose(-31.inch, -12.inch, 0.deg)
        val rightYellow = Pose(45.inch + 7.cm, -35.inch - 14.cm, 180.deg)

        val robot = Robot2(hardwareMap, telemetry, startPose.toPose2d(), true, true)
        robot.camera.setColor(DetectionPipeline.DetectionColor.RED)
        robot.camera.openCamera()
        val drive = robot.drive

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
            robot.claw.closeClaw(),
            robot.lift.goToPos(LIFT_PASS_POSE),
            ParallelAction(
                robot.arm.goToPos(0.0),
                robot.claw.goToAngle(CLAW_RAMP_ANGLE)
            ),
            robot.lift.goToPos(0),
            drive.actionBuilder(leftPurple.toPose2d())
                .setTangent(50.deg)
                .splineToSplineHeading(leftLeavePos, -8.deg)
                .lineToX(25.inch)
                .afterTime(0.sec, SequentialAction(
                    robot.lift.goToPos(LIFT_SCORE_POSE),
                    ParallelAction(
                        robot.arm.goToPos(ARM_SCORE_POS),
                        robot.claw.goToAngle(CLAW_SCORE_ANGLE)
                    ),
                ))
                .splineToSplineHeading(leftYellow, 0.deg)
                .build(),
            drive.actionBuilder(leftYellow.toPose2d())
                .setTangent(0.deg)
                .lineToX(leftYellow.position.x + 2.inch)
                .wait(0.5.sec)
                .stopAndAdd(robot.claw.openRightClaw())
                .setTangent(180.deg)
                .lineToX(leftYellow.position.x - 2.inch)
                .stopAndAdd(SequentialAction(
                    robot.claw.closeClaw(),
                    ParallelAction(
                        robot.arm.goToPos(0.0),
                        robot.claw.goToAngle(CLAW_RAMP_ANGLE)
                    ),
                    robot.lift.goToPos(0)
                ))
                .setTangent(-90.deg)
                .lineToY(-18.inch)
                .setTangent(0.deg)
                .lineToX(leftYellow.position.x + 2.inch)
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
            robot.claw.closeClaw(),
            robot.lift.goToPos(LIFT_PASS_POSE),
            ParallelAction(
                robot.arm.goToPos(0.0),
                robot.claw.goToAngle(CLAW_RAMP_ANGLE)
            ),
            robot.lift.goToPos(0),
            drive.actionBuilder(middlePurple.toPose2d())
                .setTangent(110.deg)
                .splineToSplineHeading(middleLeavePos, 5.deg)
                .lineToX(25.inch)
                .afterTime(0.sec, SequentialAction(
                    robot.lift.goToPos(LIFT_SCORE_POSE),
                    ParallelAction(
                        robot.arm.goToPos(ARM_SCORE_POS),
                        robot.claw.goToAngle(CLAW_SCORE_ANGLE)
                    ),
                ))
                .splineToSplineHeading(middleYellow, 0.deg)
                .build(),
            drive.actionBuilder(middleYellow.toPose2d())
                .setTangent(0.deg)
                .lineToX(middleYellow.position.x + 2.inch)
                .wait(0.5.sec)
                .stopAndAdd(robot.claw.openRightClaw())
                .setTangent(180.deg)
                .lineToX(middleYellow.position.x - 2.inch)
                .stopAndAdd(SequentialAction(
                    robot.claw.closeClaw(),
                    ParallelAction(
                        robot.arm.goToPos(0.0),
                        robot.claw.goToAngle(CLAW_RAMP_ANGLE)
                    ),
                    robot.lift.goToPos(0)
                ))
                .setTangent(-90.deg)
                .lineToY(-18.inch)
                .setTangent(0.deg)
                .lineToX(middleYellow.position.x + 2.inch)
                .build()
        )

        val actionRight = SequentialAction(
            drive.actionBuilder(startPose.toPose2d())
                .strafeToLinearHeading(rightPurple.position, rightPurple.heading)
                .build(),
            SequentialAction(
                robot.lift.goToPos(LIFT_PASS_POSE),
                ParallelAction(
                    robot.arm.goToPos(ARM_PIXEL_DROP_POSE),
                    robot.claw.goToAngle(CLAW_PIXEL_DROP_ANGLE)
                ),
                robot.lift.goToPos(LIFT_PIXEL_DROP_POSE)
            ),
            robot.claw.openLeftClaw(),
            robot.claw.closeClaw(),
            robot.lift.goToPos(LIFT_PASS_POSE),
            ParallelAction(
                robot.arm.goToPos(0.0),
                robot.claw.goToAngle(CLAW_RAMP_ANGLE)
            ),
            robot.lift.goToPos(0),
            drive.actionBuilder(rightPurple.toPose2d())
                .setTangent(135.deg)
                .splineToSplineHeading(rightLeavePos, -6.deg)
                .lineToX(25.inch)
                .afterTime(0.sec, SequentialAction(
                    robot.lift.goToPos(LIFT_SCORE_POSE),
                    ParallelAction(
                        robot.arm.goToPos(ARM_SCORE_POS),
                        robot.claw.goToAngle(CLAW_SCORE_ANGLE)
                    ),
                ))
                .splineToSplineHeading(rightYellow, 0.deg)
                .build(),
            drive.actionBuilder(rightYellow.toPose2d())
                .setTangent(0.deg)
                .lineToX(rightYellow.position.x + 2.inch)
                .wait(0.5.sec)
                .stopAndAdd(robot.claw.openRightClaw())
                .setTangent(180.deg)
                .lineToX(rightYellow.position.x - 2.inch)
                .stopAndAdd(SequentialAction(
                    robot.claw.closeClaw(),
                    ParallelAction(
                        robot.arm.goToPos(0.0),
                        robot.claw.goToAngle(CLAW_RAMP_ANGLE)
                    ),
                    robot.lift.goToPos(0)
                ))
                .setTangent(-90.deg)
                .lineToY(-18.inch)
                .setTangent(0.deg)
                .lineToX(rightYellow.position.x + 2.inch)
                .build()
        )

        waitForStart()

        val action = actionMiddle

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