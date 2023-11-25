package org.firstinspires.ftc.teamcode.sensor

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DistanceSensor
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit

@TeleOp
class MRRangeSensorTest : LinearOpMode() {
    override fun runOpMode() {
        val sensor = hardwareMap.get(DistanceSensor::class.java, "sensor")

        waitForStart()

        while (opModeIsActive()) {
            telemetry.addData("range", String.format("%.01f mm", sensor.getDistance(DistanceUnit.MM)))
            telemetry.update()
        }
    }
}