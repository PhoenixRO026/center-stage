package org.firstinspires.ftc.teamcode.kickoff.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.gamepad.ButtonReader;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.kickoff.robot.Robot;

@Disabled
@TeleOp
public class BurgerKingDrive extends LinearOpMode {

    ButtonReader liftLimitButton;
    ButtonReader resetHeadingButton;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        liftLimitButton = new ButtonReader(() -> gamepad1.x || gamepad2.x);
        resetHeadingButton = new ButtonReader(() -> gamepad1.b);

        Robot robot = new Robot(hardwareMap,  telemetry);

        waitForStart();

        while (opModeIsActive()) {
            robot.update();
            liftLimitButton.readValue();
            resetHeadingButton.readValue();

            robot.drive.setSniperMode(gamepad1.left_trigger >= 0.2);

            if (resetHeadingButton.wasJustPressed()) {
                robot.drive.resetFieldCentric();
                gamepad1.rumble(1.0, 1.0, 300);
            }

            robot.drive.driveFieldCentric(
                    gamepad1.left_stick_x,
                    -gamepad1.left_stick_y,
                    -gamepad1.right_stick_x
            );

            /*double input = 0.0;
            if ((-gamepad2.left_stick_y < 0.0 && robot.arm.getPosition() > Math.toRadians(90)) || (-gamepad2.left_stick_y > 0.0 && robot.arm.getPosition() < Math.toRadians(90))) {
                input = -gamepad2.left_stick_y / 2.0;
            } else {
                input = (double) -gamepad2.left_stick_y;
            }

            robot.arm.setPower(input);*/

            if (liftLimitButton.wasJustPressed()) {
                robot.lift.setLimitsDisabled(!robot.lift.getLimitsDisabled());
                gamepad1.rumble(1.0, 1.0, 300);
                gamepad2.rumble(1.0, 1.0, 300);
            }

            robot.lift.setPower(-gamepad2.right_stick_y);

            robot.claw.setPosition(gamepad2.right_trigger);

            telemetry.update();
        }
    }
}
