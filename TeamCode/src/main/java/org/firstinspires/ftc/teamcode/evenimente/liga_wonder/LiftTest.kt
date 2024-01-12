package org.firstinspires.ftc.teamcode.evenimente.liga_wonder

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.qualcomm.robotcore.eventloop.opmode.Disabled
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import org.firstinspires.ftc.teamcode.evenimente.liga_wonder.robot.CONFIG
import org.firstinspires.ftc.teamcode.evenimente.liga_wonder.robot.systems.Lift

@TeleOp
class LiftTest: LinearOpMode() {
    override fun runOpMode() {
        telemetry = MultipleTelemetry(telemetry, FtcDashboard.getInstance().telemetry)
        val lift = Lift(hardwareMap)
        /*val leftMotor = hardwareMap.get(DcMotorEx::class.java, CONFIG_ALBA.LEFT_LIFT)
        leftMotor.direction = DcMotorSimple.Direction.REVERSE
        val rightMotor = hardwareMap.get(DcMotorEx::class.java, CONFIG_ALBA.RIGHT_LIFT)
        val motors = listOf(leftMotor, rightMotor)
        motors.forEach {
            it.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
            it.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
            it.power = 0.0
        }*/

        waitForStart()

        while (opModeIsActive()) {
            lift.power = (gamepad1.right_trigger - gamepad1.left_trigger).toDouble()
            telemetry.addData("pos", lift.leftPosition)
            telemetry.update()
            /*motors.forEach {
                it.power = (gamepad1.right_trigger - gamepad1.left_trigger).toDouble()
            }*/
        }
    }
}