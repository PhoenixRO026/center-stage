package org.firstinspires.ftc.teamcode.evenimente.demo_abso;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
@TeleOp
public class BobociDrive extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        //For FTC Dashboard
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        Robot robot = new Robot(hardwareMap, new Pose2d(0, 0, Math.toRadians(90)));

        robot.initTeleOp();

        waitForStart();

        while (opModeIsActive()) {
            robot.drive.updatePoseEstimate();

            robot.sniperMode = gamepad1.left_trigger >= 0.2;

            if (gamepad1.y) {
                robot.resetFieldCentric();
            }

            robot.fieldCentricDrive(gamepad1.left_stick_x, -gamepad1.left_stick_y, gamepad1.right_stick_x);

            if (gamepad2.y) {
                robot.liftZeroPose = robot.lift.getCurrentPosition();
            }

            if (robot.lift.getCurrentPosition() > robot.liftZeroPose + Robot.liftTopLimit) {
                robot.lift.setPower(Math.min(0, -gamepad2.right_stick_y));
            } else if (robot.lift.getCurrentPosition() < robot.liftZeroPose + Robot.liftBotLimit) {
                robot.lift.setPower(Math.max(0, -gamepad2.right_stick_y));
            } else {
                robot.lift.setPower(-gamepad2.right_stick_y);
            }

            double clawPos = (Robot.closed_claw - Robot.opened_claw) * (1 - gamepad2.right_trigger) + Robot.opened_claw;
            double armPos = (Robot.arm_up - Robot.arm_down) * (1 - gamepad2.left_trigger) + Robot.arm_down;

            robot.claw.setPosition(clawPos);

            robot.arm.setPosition(armPos);

            telemetry.addData("claw pos", clawPos);
            telemetry.addData("arm pos", armPos);
            telemetry.addData("lift pos", robot.lift.getCurrentPosition());
            telemetry.addData("robot heading", robot.drive.pose.heading.log() * (180.0 / Math.PI));
            telemetry.update();
        }
        //robot.drive.pose = new Pose2d(0, 0, Math.toRadians(90));
    }

}
