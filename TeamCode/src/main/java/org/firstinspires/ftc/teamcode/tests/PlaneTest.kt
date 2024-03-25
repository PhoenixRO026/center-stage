package org.firstinspires.ftc.teamcode.tests

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.phoenix.phoenixlib.units.Time
import com.phoenix.phoenixlib.units.ms
import org.firstinspires.ftc.teamcode.systems.Plane.Companion.plane
import org.firstinspires.ftc.teamcode.systems.Plane.PlaneConfig.hold
import org.firstinspires.ftc.teamcode.systems.Plane.PlaneConfig.tiltRest

@TeleOp
class PlaneTest : LinearOpMode() {
    override fun runOpMode() {
        telemetry = MultipleTelemetry(telemetry, FtcDashboard.getInstance().telemetry)

        val plane = hardwareMap.plane()

        plane.launchPosition = hold
        plane.tiltPosition = tiltRest

        waitForStart()

        var previousTime = System.currentTimeMillis().ms
        var deltaTime: Time

        while (opModeIsActive()) {
            val now = System.currentTimeMillis().ms
            deltaTime = now - previousTime
            previousTime = now

            if (gamepad1.dpad_up) {
                plane.tiltPosition += 0.1 * deltaTime.s
            }

            if (gamepad1.dpad_down) {
                plane.tiltPosition -= 0.1 * deltaTime.s
            }

            if (gamepad1.y) {
                plane.launchPosition += 0.1 * deltaTime.s
            }

            if (gamepad1.a) {
                plane.launchPosition -= 0.1 * deltaTime.s
            }

            telemetry.addData("tilt pos", plane.tiltPosition)
            telemetry.addData("launch pos", plane.launchPosition)
            telemetry.update()
        }
    }
}