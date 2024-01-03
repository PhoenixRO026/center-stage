/*
package org.firstinspires.ftc.teamcode.evenimente.demo_abso;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;

@TeleOp
public class TeleTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, Math.toRadians(90)));
        Robot robot = new Robot(hardwareMap, new Pose2d(0, 0, Math.toRadians(90)));
        robot.initTeleOp();

        waitForStart();

        while (opModeIsActive()) {
            drive.updatePoseEstimate();

            telemetry.addData("drive heading", Math.toDegrees(drive.pose.heading.log()));
            telemetry.addData("robot heading", Math.toDegrees(robot.drive.pose.heading.log()));
            telemetry.update();
        }
    }
}
*/
