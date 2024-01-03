/*
package org.firstinspires.ftc.teamcode.evenimente.demo_abso;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous
public class BobociAuto extends LinearOpMode {
    Pose2d startPose = new Pose2d(12, -61, Math.toRadians(90));

    enum Detection {
        CASE_1,
        CASE_2,
        CASE_3
    }

    Detection detectedCase = Detection.CASE_2;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        Robot robot = new Robot(hardwareMap, startPose);
        robot.initAuto(telemetry);

        Action auto1 = new SequentialAction(
                robot.drive.actionBuilder(startPose)
                        .splineTo(new Vector2d(2, -34), Math.toRadians(125))
                        .build(),
                robot.armDown(),
                robot.openClaw(),
                robot.armUp(),
                robot.drive.actionBuilder(robot.drive.pose)
                        .turn(Math.toRadians(-125))
                        .lineToX(44)
                        .lineToY(-17)
                        .build()
        );

        Action auto2 = new SequentialAction(
                robot.drive.actionBuilder(startPose)
                        .lineToY(-61 + 29)
                        .build(),
                robot.armDown(),
                robot.openClaw(),
                robot.armUp(),
                robot.drive.actionBuilder(robot.drive.pose)
                        .turn(Math.toRadians(-90))
                        .lineToX(12 + 38)
                        .lineToY(-61 + 29 + 18)
                        .build()
        );

        Action auto3 = new SequentialAction(
                robot.drive.actionBuilder(startPose)
                        .splineTo(new Vector2d(22, -34), Math.toRadians(45))
                        .build(),
                robot.armDown(),
                robot.openClaw(),
                robot.armUp(),
                robot.drive.actionBuilder(robot.drive.pose)
                        .turn(Math.toRadians(-45))
                        .lineToX(22 + 24)
                        .lineToY(-34 + 15)
                        .build()
        );

        while (opModeInInit()) {
            switch (robot.pipeline.getAnalysis()) {
                case LEFT:
                    telemetry.addLine("LEFT");
                    detectedCase = Detection.CASE_1;
                    break;
                case CENTER:
                    telemetry.addLine("CENTER");
                    detectedCase = Detection.CASE_2;
                    break;
                case RIGHT:
                    telemetry.addLine("RIGHT");
                    detectedCase = Detection.CASE_3;
                    break;
            }
            telemetry.update();
            sleep(50);
        }

        switch (detectedCase) {
            case CASE_1:
                Actions.runBlocking(auto1);
                break;
            case CASE_2:
                Actions.runBlocking(auto2);
                break;
            case CASE_3:
                Actions.runBlocking(auto3);
                break;
        }
    }
}
*/
