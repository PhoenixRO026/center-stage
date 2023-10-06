package org.firstinspires.ftc.teamcode.kickoff;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;

@Disabled
@TeleOp(name = "Field Centric Drive")
public class FieldCentricDrive extends LinearOpMode {

    @Override
    public void runOpMode() {
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));

        waitForStart();

        while (opModeIsActive()) {
            drive.updatePoseEstimate();

            Vector2d inputVector = new Vector2d(gamepad1.left_stick_x, -gamepad1.left_stick_y);

            inputVector = UtilKt.rotate(inputVector, -drive.pose.heading.log() - Math.toRadians(90));

            PoseVelocity2d input = new PoseVelocity2d(inputVector, gamepad1.right_stick_x);

            drive.setDrivePowers(input);
        }
    }
}
