package org.firstinspires.ftc.teamcode.evenimente.alba;

import static org.firstinspires.ftc.teamcode.evenimente.alba.robot.ConstantsAlbaKt.getARM_PIXEL_DROP_POSE;
import static org.firstinspires.ftc.teamcode.evenimente.alba.robot.ConstantsAlbaKt.getCLAW_PIXEL_DROP_POSE;
import static org.firstinspires.ftc.teamcode.evenimente.alba.robot.ConstantsAlbaKt.getLIFT_PASS_POSE;
import static org.firstinspires.ftc.teamcode.evenimente.alba.robot.ConstantsAlbaKt.getLIFT_PIXEL_DROP_POSE;
import static org.firstinspires.ftc.teamcode.evenimente.liga_wonder.robot.ConstantsKt.CLAW_RAMP_POS;

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

import org.firstinspires.ftc.teamcode.evenimente.alba.robot.Robot2;
import org.firstinspires.ftc.teamcode.evenimente.liga_wonder.robot.systems.DetectionPipeline;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;

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
        robot.getCamera().setColor(DetectionPipeline.DetectionColor.RED);
        MecanumDrive drive = robot.getDrive().getDrive();

        robot.getCamera().openCamera();

        Action actionLEFT = new SequentialAction(
                new ParallelAction(
                        drive.actionBuilder(startPose)
                                .setTangent(Math.toRadians(-90.0))
                                .splineToLinearHeading(purplePixelPoseLeft, Math.toRadians(-90.0))
                                .build(),
                        new SequentialAction(
                                robot.getLift().goToPos(getLIFT_PASS_POSE()),
                                new ParallelAction(
                                        robot.getClaw().goToAngle(getCLAW_PIXEL_DROP_POSE()),
                                        robot.getArm().goToPos(getARM_PIXEL_DROP_POSE())
                                ),
                                robot.getLift().goToPos(getLIFT_PIXEL_DROP_POSE())
                        )
                ),
                robot.getClaw().openLeftClaw(),
                robot.getClaw().closeClaw(),
                robot.getLift().goToPos(getLIFT_PASS_POSE()),
                new ParallelAction(
                        robot.getArm().goToPos(0.0),
                        robot.getClaw().goToAngle(CLAW_RAMP_POS)
                ),
                robot.getLift().goToPos(0)
        );

        Action actionRIGHT = new SequentialAction(
                new ParallelAction(
                        drive.actionBuilder(startPose)
                                .setTangent(Math.toRadians(-90.0))
                                .splineToLinearHeading(purplePixelPoseRight, Math.toRadians(-90.0))
                                .build(),
                        new SequentialAction(
                                robot.getLift().goToPos(getLIFT_PASS_POSE()),
                                new ParallelAction(
                                        robot.getClaw().goToAngle(getCLAW_PIXEL_DROP_POSE()),
                                        robot.getArm().goToPos(getARM_PIXEL_DROP_POSE())
                                ),
                                robot.getLift().goToPos(getLIFT_PIXEL_DROP_POSE())
                        )
                ),
                robot.getClaw().openLeftClaw(),
                robot.getClaw().closeClaw(),
                robot.getLift().goToPos(getLIFT_PASS_POSE()),
                new ParallelAction(
                        robot.getArm().goToPos(0.0),
                        robot.getClaw().goToAngle(CLAW_RAMP_POS)
                ),
                robot.getLift().goToPos(0)
        );

        Action actionMIDDLE = new SequentialAction(
                new ParallelAction(
                        drive.actionBuilder(startPose)
                                .setTangent(Math.toRadians(-90.0))
                                .splineToLinearHeading(purplePixelPoseMidle, Math.toRadians(-90.0))
                                .build(),
                        new SequentialAction(
                                robot.getLift().goToPos(getLIFT_PASS_POSE()),
                                new ParallelAction(
                                        robot.getClaw().goToAngle(getCLAW_PIXEL_DROP_POSE()),
                                        robot.getArm().goToPos(getARM_PIXEL_DROP_POSE())
                                ),
                                robot.getLift().goToPos(getLIFT_PIXEL_DROP_POSE())
                        )
                ),
                robot.getClaw().openLeftClaw(),
                robot.getClaw().closeClaw(),
                robot.getLift().goToPos(getLIFT_PASS_POSE()),
                new ParallelAction(
                        robot.getArm().goToPos(0.0),
                        robot.getClaw().goToAngle(CLAW_RAMP_POS)
                ),
                robot.getLift().goToPos(0)
        );

        while (opModeInInit()) {
            robot.getCamera().displayDetection();
            telemetry.update();
            sleep(10);
        }

        Action action = actionLEFT;

        switch (robot.getCamera().getDetectionPosition()) {
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
