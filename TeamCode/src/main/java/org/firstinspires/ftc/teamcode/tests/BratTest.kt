package org.firstinspires.ftc.teamcode.tests

import com.qualcomm.robotcore.eventloop.opmode.Disabled
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.lib.hardware.servo.RangedServo.Companion.rangedServo
import com.phoenix.phoenixlib.units.DeltaTime

@Disabled
@TeleOp(group = "Debug")
class BratTest : LinearOpMode() {

    data object BratTestConfig {
        @JvmField var cutieOffset: Double = 0.02
    }

    override fun runOpMode() {
        val rightRange = BratTestConfig.cutieOffset..1.0
        val leftRange = 0.0..(1.0 - BratTestConfig.cutieOffset)
        val leftServo = hardwareMap.rangedServo("leftArm", range = leftRange)
        val rightServo = hardwareMap.rangedServo("rightArm", range = rightRange)
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

            telemetry.addData("position", position)
            telemetry.update()
        }
    }
}