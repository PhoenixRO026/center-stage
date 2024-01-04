package org.firstinspires.ftc.teamcode.evenimente.liga_wonder

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.teamcode.evenimente.liga_wonder.robot.Robot

@Autonomous
class RevAuto : LinearOpMode() {
    override fun runOpMode() {
        telemetry = MultipleTelemetry(telemetry, FtcDashboard.getInstance().telemetry)
        val robot = Robot(hardwareMap, telemetry)

        while (opModeInInit()) {
            //robot.camera.displayDetection()
            telemetry.update()
            sleep(50)
        }

        while (isStarted && !isStopRequested) {
            robot.update()
        }
    }
}