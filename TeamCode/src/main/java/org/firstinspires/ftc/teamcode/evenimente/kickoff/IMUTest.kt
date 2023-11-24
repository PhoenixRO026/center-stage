package org.firstinspires.ftc.teamcode.evenimente.kickoff

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot
import com.qualcomm.robotcore.eventloop.opmode.Disabled
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.IMU
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.teamcode.evenimente.kickoff.robot.IMU_ID

@Disabled
@TeleOp
class IMUTest : LinearOpMode() {
    override fun runOpMode() {
        val imu: IMU = hardwareMap.get(IMU::class.java, IMU_ID)
        val params: IMU.Parameters = IMU.Parameters(
                RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
                )
        )
        imu.initialize(params)

        val motor: DcMotorEx = hardwareMap.get(DcMotorEx::class.java, "motor")

        waitForStart()

        while (opModeIsActive()) {
            val angle = imu.robotYawPitchRollAngles.getRoll(AngleUnit.DEGREES)
            motor.power = angle / 90

            telemetry.addData("roll angle", angle)
            telemetry.update()
        }
    }
}