package org.firstinspires.ftc.teamcode.tests

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.systems.Lift
import org.firstinspires.ftc.teamcode.systems.Lift.Companion.lift

@TeleOp
class LiftTuning : LinearOpMode() {

    @Config
    data object LiftTuning {
        @JvmField var pos = 0
    }

    override fun runOpMode() {
        telemetry = MultipleTelemetry(telemetry, FtcDashboard.getInstance().telemetry)

        val lift = hardwareMap.lift()

        waitForStart()

        lift.mode = Lift.MODE.TARGET

        while (opModeIsActive()) {
            lift.targetPositionTicks = LiftTuning.pos

            lift.update()

            telemetry.addData("lift pos", lift.positionTicks)
            telemetry.addData("lift target", lift.targetPositionTicks)
            telemetry.update()
        }
    }
}