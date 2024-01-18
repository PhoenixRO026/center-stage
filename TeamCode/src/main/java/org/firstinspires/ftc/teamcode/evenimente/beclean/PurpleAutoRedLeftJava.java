package org.firstinspires.ftc.teamcode.evenimente.beclean;

import static org.firstinspires.ftc.teamcode.evenimente.beclean.robot.ConstantsKt.ARM_PIXEL_DROP_POSE;
import static org.firstinspires.ftc.teamcode.evenimente.beclean.robot.ConstantsKt.CLAW_PIXEL_DROP_ANGLE;
import static org.firstinspires.ftc.teamcode.evenimente.beclean.robot.ConstantsKt.CLAW_RAMP_ANGLE;
import static org.firstinspires.ftc.teamcode.evenimente.beclean.robot.ConstantsKt.LIFT_PASS_POSE;
import static org.firstinspires.ftc.teamcode.evenimente.beclean.robot.ConstantsKt.LIFT_PIXEL_DROP_POSE;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.evenimente.beclean.robot.Robot2;
import org.firstinspires.ftc.teamcode.evenimente.beclean.robot.systems.DetectionPipeline;
import org.firstinspires.ftc.teamcode.evenimente.beclean.robot.systems.Drive2;

@Autonomous
public class PurpleAutoRedLeftJava extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        FtcDashboard dash = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        Pose2d startPose = new Pose2d(-36.0, 61.0, Math.toRadians(90.0));
        Pose2d purplePixelPoseLeft = new Pose2d(-34.0, 44.0, Math.toRadians(135.0));
        Pose2d purplePixelPoseRight = new Pose2d(-30.0 - 6 + 2, 39.0, Math.toRadians(20.0));
        Pose2d purplePixelPoseMidle = new Pose2d(-29.0,40.0, Math.toRadians(90.0));

        Robot2 robot = new Robot2(hardwareMap, telemetry, startPose, false, true);
        robot.camera.setColor(DetectionPipeline.DetectionColor.RED);
        Drive2 drive = robot.drive/*.getDrive()*/;

        robot.camera.openCamera();

        Action actionLEFT = new SequentialAction(
                new ParallelAction(
                        drive.actionBuilder(startPose)
                                .setTangent(Math.toRadians(-90.0))
                                .splineToLinearHeading(purplePixelPoseLeft, Math.toRadians(-90.0))
                                .build(),
                        new SequentialAction(
                                robot.lift.goToPos(LIFT_PASS_POSE),
                                new ParallelAction(
                                        robot.claw.goToAngle(CLAW_PIXEL_DROP_ANGLE),
                                        robot.arm.goToPos(ARM_PIXEL_DROP_POSE)
                                ),
                                robot.lift.goToPos(LIFT_PIXEL_DROP_POSE)
                        )
                ),
                robot.claw.openLeftClaw(),
                robot.claw.closeClaw(),
                robot.lift.goToPos(LIFT_PASS_POSE),
                new ParallelAction(
                        robot.arm.goToPos(0.0),
                        robot.claw.goToAngle(CLAW_RAMP_ANGLE)
                ),
                robot.lift.goToPos(0)
        );

        Action actionRIGHT = new SequentialAction(
                new ParallelAction(
                        drive.actionBuilder(startPose)
                                .setTangent(Math.toRadians(-90.0))
                                .splineToLinearHeading(purplePixelPoseRight, Math.toRadians(-90.0))
                                .build(),
                        new SequentialAction(
                                robot.lift.goToPos(LIFT_PASS_POSE),
                                new ParallelAction(
                                        robot.claw.goToAngle(CLAW_PIXEL_DROP_ANGLE),
                                        robot.arm.goToPos(ARM_PIXEL_DROP_POSE)
                                ),
                                robot.lift.goToPos(LIFT_PIXEL_DROP_POSE)
                        )
                ),
                robot.claw.openLeftClaw(),
                robot.claw.closeClaw(),
                robot.lift.goToPos(LIFT_PASS_POSE),
                new ParallelAction(
                        robot.arm.goToPos(0.0),
                        robot.claw.goToAngle(CLAW_RAMP_ANGLE)
                ),
                robot.lift.goToPos(0)
        );

        Action actionMIDDLE = new SequentialAction(
                new ParallelAction(
                        drive.actionBuilder(startPose)
                                .setTangent(Math.toRadians(-90.0))
                                .splineToLinearHeading(purplePixelPoseMidle, Math.toRadians(-90.0))
                                .build(),
                        new SequentialAction(
                                robot.lift.goToPos(LIFT_PASS_POSE),
                                new ParallelAction(
                                        robot.claw.goToAngle(CLAW_PIXEL_DROP_ANGLE),
                                        robot.arm.goToPos(ARM_PIXEL_DROP_POSE)
                                ),
                                robot.lift.goToPos(LIFT_PIXEL_DROP_POSE)
                        )
                ),
                robot.claw.openLeftClaw(),
                robot.claw.closeClaw(),
                robot.lift.goToPos(LIFT_PASS_POSE),
                new ParallelAction(
                        robot.arm.goToPos(0.0),
                        robot.claw.goToAngle(CLAW_RAMP_ANGLE)
                ),
                robot.lift.goToPos(0)
        );

        while (opModeInInit()) {
            robot.camera.displayDetection();
            telemetry.update();
            sleep(10);
        }

        Action action = actionLEFT;

        switch (robot.camera.getDetectionPosition()) {
            case LEFT:
                break;
            case CENTER:
                action = actionMIDDLE;
                break;
            case RIGHT:
                action = actionRIGHT;
                break;
        }

        Canvas c = new Canvas();
        action.preview(c);
        boolean running = true;

        while (isStarted() && !isStopRequested() && running) {
            robot.update();

            TelemetryPacket p = new TelemetryPacket();
            p.fieldOverlay().getOperations().addAll(c.getOperations());

            running = action.run(p);

            dash.sendTelemetryPacket(p);

            telemetry.update();
        }
    }
}
