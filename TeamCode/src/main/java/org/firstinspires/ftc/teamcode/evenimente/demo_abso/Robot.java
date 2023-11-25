package org.firstinspires.ftc.teamcode.evenimente.demo_abso;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Config
public class Robot {
    MecanumDrive drive;
    DcMotorEx lift;
    Servo arm;
    Servo claw;
    OpenCvCamera webcam;
    DetectionPipeline pipeline;
    public static Pose2d teleOpPose = new Pose2d(0, 0, Math.toRadians(90));

    public Action moveArm(double position) {
        return new Action() {
            final ElapsedTime time = new ElapsedTime();
            boolean init = true;
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if (init) {
                    init = false;
                    claw.setPosition(position);
                }
                return time.seconds() < 0.5;
            }
        };
    }

    public Action armDown() {
        return moveArm(arm_down);
    }

    public Action armUp() {
        return moveArm(arm_up);
    }

    public Action moveClaw(double position) {
        return new Action() {
            final ElapsedTime time = new ElapsedTime();
            boolean init = true;
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if (init) {
                    init = false;
                    claw.setPosition(position);
                }
                return time.seconds() < 0.5;
            }
        };
    }

    public Action closeClaw() {
        return moveClaw(closed_claw);
    }

    public Action openClaw() {
        return moveClaw(opened_claw);
    }

    public Action moveLift(int position) {
        return new Action() {
            boolean init = true;
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if (init) {
                    init = false;
                    lift.setTargetPosition(position);
                }
                return lift.isBusy();
            }
        };
    }

    public static double closed_claw = 0.23;
    public static double opened_claw = 0;
    public static double arm_up = 0.12;
    public static double arm_down = 0.079;
    public static double arm_init = 0.11;
    public boolean sniperMode = false;
    public static double sniperSpeed = 0.35;
    public static int liftTopLimit = 1500;
    public static int liftBotLimit = 50;
    public double liftZeroPose = 0;

    void initTeleOp(HardwareMap hardwareMap) {
        drive = new MecanumDrive(hardwareMap, teleOpPose);
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
    }

    void initAuto(HardwareMap hardwareMap, Pose2d beginPose, Telemetry telemetry) {
        drive = new MecanumDrive(hardwareMap, beginPose);
        lift = hardwareMap.get(DcMotorEx.class, "lift");
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setTargetPosition(0);
        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lift.setDirection(DcMotorSimple.Direction.FORWARD);
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lift.setPower(1);
        arm = hardwareMap.get(Servo.class, "arm");
        claw = hardwareMap.get(Servo.class, "claw");
        arm.setPosition(arm_init);
        claw.setPosition(closed_claw);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        pipeline = new DetectionPipeline();
        webcam.setPipeline(pipeline);

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {
                telemetry.addLine("Failed to open camera");
                telemetry.update();
            }
        });
    }

    void resetFieldCentric() {
        drive.pose = new Pose2d(drive.pose.position.x, drive.pose.position.y, Math.toRadians(90));
    }

    void fieldCentricDrive(double x, double y, double heading) {
        double robot_heading = drive.pose.heading.log() * (180.0 / Math.PI);

        //noinspection SuspiciousNameCombination
        com.arcrobotics.ftclib.geometry.Vector2d input = new com.arcrobotics.ftclib.geometry.Vector2d(y, -x);

        input = input.rotateBy(-robot_heading + 90);

        double newHeading = -heading;

        if (sniperMode) {
            newHeading *= sniperSpeed;
            input = input.times(sniperSpeed);
        }

        drive.setDrivePowers(new PoseVelocity2d(new Vector2d(input.getX(), input.getY()), newHeading));
    }
}
