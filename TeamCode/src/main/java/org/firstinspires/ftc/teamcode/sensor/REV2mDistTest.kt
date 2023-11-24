package org.firstinspires.ftc.teamcode.sensor

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.qualcomm.hardware.rev.Rev2mDistanceSensor
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.Disabled
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit

@Disabled
@Autonomous
class REV2mDistTest : LinearOpMode() {
    override fun runOpMode() {
        telemetry = MultipleTelemetry(telemetry, FtcDashboard.getInstance().telemetry)
        val sensor = hardwareMap.get(Rev2mDistanceSensor::class.java, "dist")
        val kalmanSensor = KalmanDistanceSensor(sensor)

        waitForStart()

        while (opModeIsActive()) {
            telemetry.addData("raw range mm", sensor.getDistance(DistanceUnit.MM))
            telemetry.addData("kalman range mm", kalmanSensor.position)
            telemetry.update()
        }
    }
}