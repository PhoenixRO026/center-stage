package org.firstinspires.ftc.teamcode.tests

import com.acmerobotics.dashboard.config.Config
import com.phoenix.phoenixlib.units.DeltaTime
import com.qualcomm.robotcore.eventloop.opmode.Disabled
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.CRServo
import com.qualcomm.robotcore.hardware.Servo
import org.firstinspires.ftc.teamcode.lib.hardware.servo.RangedServo.Companion.rangedServo

@Disabled
@TeleOp
class CutieTest : LinearOpMode() {

    @Config
    data object CutieTestConfig {
        @JvmField var cutieOffset: Double = 0.08
    }

    override fun runOpMode() {
        val leftRange = CutieTestConfig.cutieOffset..1.0
        val rightRange = 0.0..(1.0 - CutieTestConfig.cutieOffset)
        val leftServo = hardwareMap.rangedServo("leftBox", Servo.Direction.REVERSE, leftRange)
        val rightServo = hardwareMap.rangedServo("rightBox", range = rightRange)
        val wheel = hardwareMap.get(CRServo::class.java, "wheelBox")
        val time = DeltaTime()

        var position = 0.5

        leftServo.position = position
        rightServo.position = position

        waitForStart()

        while (opModeIsActive()) {
            position += gamepad1.left_stick_x * time.calculateDeltaTime().s * 0.5
            position = position.coerceIn(0.0, 1.0)

            if (gamepad1.a) {
                position = 0.5
            }

            leftServo.position = position
            rightServo.position = position

            wheel.power = gamepad1.right_stick_x.toDouble()

            telemetry.addData("position", position)
            telemetry.update()
        }
    }
}