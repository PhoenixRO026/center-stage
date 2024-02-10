package org.firstinspires.ftc.teamcode.regionala

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.lib.units.Time
import org.firstinspires.ftc.teamcode.lib.units.ms
import org.firstinspires.ftc.teamcode.robot.Plane
import org.firstinspires.ftc.teamcode.robot.Plane.Companion.hold
import org.firstinspires.ftc.teamcode.robot.Plane.Companion.plane

@TeleOp
class PlaneTest : LinearOpMode() {
    override fun runOpMode() {
        telemetry = MultipleTelemetry(telemetry, FtcDashboard.getInstance().telemetry)

        val plane = hardwareMap.plane()

        plane.launchPosition = hold
        plane.tiltPosition = Plane.tiltRest

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