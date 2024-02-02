package org.firstinspires.ftc.teamcode.evenimente.beclean

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.canvas.Canvas
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.acmerobotics.roadrunner.InstantAction
import com.acmerobotics.roadrunner.ParallelAction
import com.acmerobotics.roadrunner.Pose2d
import com.acmerobotics.roadrunner.SequentialAction
import com.acmerobotics.roadrunner.SleepAction
import com.phoenix_ro026.phoenixlib.units.Distance2d
import com.phoenix_ro026.phoenixlib.units.Pose
import com.phoenix_ro026.phoenixlib.units.cm
import com.phoenix_ro026.phoenixlib.units.deg
import com.phoenix_ro026.phoenixlib.units.inch
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
import org.firstinspires.ftc.teamcode.evenimente.regio.robot.Robot4

@Autonomous
class AleIncearcaChestiiK : LinearOpMode() {

    override fun runOpMode() {
        val dash = FtcDashboard.getInstance()
        telemetry = MultipleTelemetry(telemetry, dash.telemetry)
        val startPose = Pose(12.inch, -61.inch, -90.deg)
        val leftPurple = Pose(7.inch, -38.inch, -45.deg)
        val leftYellow = Pose(46.inch, -32.inch, 180.deg)
        val midBoard = Pose(46.inch, -34.inch, 180.deg)
        val stacky = Pose(-58.inch, -34.inch, 180.deg)
        val stacky2 = Pose(-58.inch, -38.5.inch, 180.deg)

        val robot = Robot4(hardwareMap, telemetry, startPose.toPose2d(), true, true)
        robot.camera.setColor(DetectionPipeline.DetectionColor.RED)
        robot.camera.openCamera()
        val drive = robot.drive

        val actionLeft = SequentialAction(
                /*SequentialAction(
                        robot.lift.goToPos(LIFT_PASS_POSE),
                        /*ParallelAction(
                                robot.arm.goToPos(ARM_PIXEL_DROP_POSE),
                                robot.claw.goToAngle(CLAW_PIXEL_DROP_ANGLE)
                        ),
                        robot.lift.goToPos(LIFT_PIXEL_DROP_POSE)*/
                ),*/
                drive.actionBuilder(startPose.toPose2d())
                        .strafeToLinearHeading(leftPurple.position, leftPurple.heading)
                        .build(),
                        ParallelAction (
                        robot.lift.goToPos(LIFT_PASS_POSE),
                        robot.arm.goToPos(0.0),
                        ),
                drive.actionBuilder(leftPurple.toPose2d())
                        .setTangent(-45.deg)
                        .lineToY (-45.inch)
                        .setTangent (-90.deg)
                        .strafeToLinearHeading(leftYellow.position, leftYellow.heading)
                        .wait (1.sec)
                        .build(),
                ParallelAction(
                        robot.arm.goToPos(ARM_SCORE_POS),
                        robot.claw.goToAngle(CLAW_SCORE_ANGLE)
                ),
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

                        ))
                        .afterTime(0.sec, robot.lift.goToPos(0))
                //drive.actionBuilder(leftYellow.toPose2d())
                        .setTangent(-90.deg)
                        .splineToSplineHeading(Pose(24.inch, -59.inch, 180.deg), 180.deg)
                        /*.lineToY(-59.inch)
                        .wait(1.sec)*/
                        //.setTangent(0.deg)
                        .lineToX (-24.inch)
                        .splineTo(stacky.position, 180.deg)
                        .build(),
                        SequentialAction(
                            robot.claw.openClawRamp(),
                                InstantAction { robot.intake.power = 1.0 },
                                robot.intake.goToAngle(0.535),
                                SleepAction(3.0),
                                robot.intake.goToAngle(0.0),
                                drive.actionBuilder(stacky.toPose2d())
                                        .setTangent(Math.toRadians(-90.0))
                                        .lineToY(-38.5.inch)
                                        .build(),
                                SleepAction(1.0),
                                robot.intake.goToAngle(0.64),
                                SleepAction(1.0),
                                robot.intake.goToAngle(0.0),
                                InstantAction { robot.intake.power = 0.0 },
                                robot.claw.closeClaw()
                        ),
                        drive.actionBuilder (stacky2.toPose2d())
                        .setTangent(0.deg)
                        .splineToConstantHeading(Distance2d(-30.0.inch, -57.0.inch), 0.0.deg)
                        .lineToX (18.inch)
                        .afterTime(0.sec, robot.lift.goToPos(0))
                        .splineTo(midBoard.position, 0.deg)
                        .wait(1.sec)
                        .build(),
                        robot.arm.goToPos(0.0),
                ParallelAction(
                        robot.arm.goToPos(ARM_SCORE_POS),
                        robot.claw.goToAngle(CLAW_SCORE_ANGLE)
                ),
                drive.actionBuilder(midBoard.toPose2d())
                        .setTangent(0.deg)
                        .lineToX(midBoard.position.x + 2.inch)
                        .wait(0.5.sec)
                        .stopAndAdd(robot.claw.openRightClaw())
                        .setTangent(180.deg)
                        .lineToX(midBoard.position.x - 2.inch)
                        .stopAndAdd(SequentialAction(
                                robot.claw.closeClaw(),
                                ParallelAction(
                                        robot.arm.goToPos(0.0),
                                        robot.claw.goToAngle(CLAW_RAMP_ANGLE)
                                ),
                                robot.lift.goToPos(0)
                        ))
                        .build()
        )

        waitForStart()

        val action = actionLeft

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