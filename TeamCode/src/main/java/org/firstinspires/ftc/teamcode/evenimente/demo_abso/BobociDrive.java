package org.firstinspires.ftc.teamcode.evenimente.demo_abso;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;

@Config
@TeleOp
public class BobociDrive extends LinearOpMode {
    MecanumDrive drive;
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

    @Override
    public void runOpMode() throws InterruptedException {
        //For FTC Dashboard
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
        lift = hardwareMap.get(DcMotorEx.class, "lift");
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lift.setDirection(DcMotorSimple.Direction.FORWARD);
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lift.setPower(0);
        arm = hardwareMap.get(Servo.class, "arm");
        claw = hardwareMap.get(Servo.class, "claw");
        arm.setPosition(arm_init);
        claw.setPosition(closed_claw);

        waitForStart();

        while (opModeIsActive()) {
            drive.updatePoseEstimate();

            sniperMode = gamepad1.left_trigger >= 0.2;

            if (gamepad1.y) {
                resetFieldCentric();
            }

            fieldCentricDrive(gamepad1.left_stick_x, -gamepad1.left_stick_y, gamepad1.right_stick_x);

            if (gamepad2.y) {
                liftZeroPose = lift.getCurrentPosition();
            }

            if (lift.getCurrentPosition() > liftZeroPose + liftTopLimit) {
                lift.setPower(Math.min(0, -gamepad2.right_stick_y));
            } else if (lift.getCurrentPosition() < liftZeroPose + liftBotLimit) {
                lift.setPower(Math.max(0, -gamepad2.right_stick_y));
            } else {
                lift.setPower(-gamepad2.right_stick_y);
            }

            double clawPos = (closed_claw - opened_claw) * (1 - gamepad2.right_trigger) + opened_claw;
            double armPos = (arm_up - arm_down) * (1 - gamepad2.left_trigger) + arm_down;

            claw.setPosition(clawPos);

            arm.setPosition(armPos);

            telemetry.addData("claw pos", clawPos);
            telemetry.addData("arm pos", armPos);
            telemetry.addData("lift pos", lift.getCurrentPosition());
            telemetry.addData("robot heading", drive.pose.heading.log() * (180.0 / Math.PI));
            telemetry.update();
        }
    }
    void resetFieldCentric() {
        drive.pose = new Pose2d(drive.pose.position.x, drive.pose.position.y, 0);
    }

    void fieldCentricDrive(double x, double y, double heading) {
        double robot_heading = drive.pose.heading.log() * (180.0 / Math.PI);

        //noinspection SuspiciousNameCombination
        com.arcrobotics.ftclib.geometry.Vector2d input = new com.arcrobotics.ftclib.geometry.Vector2d(y, -x);

        input = input.rotateBy(-robot_heading);

        double newHeading = -heading;

        if (sniperMode) {
            newHeading *= sniperSpeed;
            input = input.times(sniperSpeed);
        }

        drive.setDrivePowers(new PoseVelocity2d(new Vector2d(input.getX(), input.getY()), newHeading));
    }
}
