package org.firstinspires.ftc.teamcode.evenimente.liga_wonder

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.IMU
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive

@TeleOp
class IMUTest: LinearOpMode() {
    override fun runOpMode() {
        telemetry = MultipleTelemetry(telemetry, FtcDashboard.getInstance().telemetry)
        val parameters = IMU.Parameters(
            RevHubOrientationOnRobot(
                MecanumDrive.PARAMS.logoFacingDirection, MecanumDrive.PARAMS.usbFacingDirection
            )
        )
        val imu = hardwareMap.get(IMU::class.java, "imu")
        imu.resetYaw()
        imu.initialize(parameters)

        waitForStart()

        while (opModeIsActive()) {
            val heading = imu.robotYawPitchRollAngles.getYaw(AngleUnit.DEGREES)
            telemetry.addData("heading", heading)
            telemetry.update()
        }
    }
}