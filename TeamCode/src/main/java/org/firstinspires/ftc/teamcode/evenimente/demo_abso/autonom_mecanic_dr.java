package org.firstinspires.ftc.teamcode.evenimente.demo_abso;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;

@Autonomous
public class autonom_mecanic_dr extends LinearOpMode {
    DcMotorEx lift;
    Servo arm;
    Servo claw;

    public static double closed_claw = 0.23;
    public static double opened_claw = 0;
    public static double arm_up = 0.095;
    public static double arm_down = 0.055;
    public static double arm_init = 0.11;
    boolean sniperMode = false;
    public static double sniperSpeed = 0.35;
    public static int liftTopLimit = 1500;
    public static int liftBotLimit = 50;
    double liftZeroPose = 0;
    Pose2d initPose = new Pose2d(12, -61, Math.toRadians(90));
    MecanumDrive drive = new MecanumDrive(hardwareMap, initPose);
    @Override
    public void runOpMode() throws InterruptedException {
        Action traj1 = drive.actionBuilder(initPose)
                    .lineToY(-61 + 25)
                    .turn(Math.toRadians(60))
                    .waitSeconds(1)
                    .splineTo(new Vector2d(47, -35), Math.toRadians(0))
                    .waitSeconds(0.5)
                    .lineToYConstantHeading(-35 + 18)
                    .build();
        Action traj2 = drive.actionBuilder(initPose)
                .lineToY(-61 + 25)
                .turn(Math.toRadians(60))
                .waitSeconds(1)
                .splineTo(new Vector2d(47, -35), Math.toRadians(0))
                .waitSeconds(0.5)
                .lineToYConstantHeading(-35 + 18)
                .build();
        Action traj3 = drive.actionBuilder(initPose)
                .splineTo(new Vector2d(22, -34), Math.toRadians(45))
                .setTangent(45)
                .

    }
}