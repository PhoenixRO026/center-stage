package org.firstinspires.ftc.teamcode.test.multi

import com.outoftheboxrobotics.photoncore.Photon
import com.qualcomm.hardware.rev.Rev2mDistanceSensor
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotorEx
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.firstinspires.ftc.teamcode.lib.opmode.MultiThreadOpMode
import org.firstinspires.ftc.teamcode.lib.units.ms
import org.firstinspires.ftc.teamcode.lib.units.s

@Photon
@TeleOp
class PhotonMultiTest : MultiThreadOpMode(){

    var deltaTime2 = 20.ms

    var distance = 0.0

    override fun sideRunOpMode() {
        val motor5 = hardwareMap.get(DcMotorEx::class.java, "motor5")
        val motor6 = hardwareMap.get(DcMotorEx::class.java, "motor6")
        val motor7 = hardwareMap.get(DcMotorEx::class.java, "motor7")
        val motor8 = hardwareMap.get(DcMotorEx::class.java, "motor8")

        val distSens = hardwareMap.get(Rev2mDistanceSensor::class.java, "dist")

        val motors = listOf(motor5, motor6, motor7, motor8)

        var previousTime = System.currentTimeMillis().ms

        waitForStart()

        while (isStarted && !isStopRequested) {
            val now = System.currentTimeMillis().ms
            deltaTime2 = now - previousTime
            previousTime = now

            motors.forEach {
                it.power = Math.random()
            }

            distance = distSens.getDistance(DistanceUnit.MM)

            /*telemetry.addData("expansion fps", 1.s / deltaTime)
            telemetry.update()*/
        }
    }

    override fun mainRunOpMode() {
        val motor1 = hardwareMap.get(DcMotorEx::class.java, "motor1")
        val motor2 = hardwareMap.get(DcMotorEx::class.java, "motor2")
        val motor3 = hardwareMap.get(DcMotorEx::class.java, "motor3")
        val motor4 = hardwareMap.get(DcMotorEx::class.java, "motor4")

        val motors = listOf(motor1, motor2, motor3, motor4)

        var previousTime = System.currentTimeMillis().ms

        waitForStart()

        while (isStarted && !isStopRequested) {
            val now = System.currentTimeMillis().ms
            val deltaTime = now - previousTime
            previousTime = now

            motors.forEach {
                it.power = Math.random()
            }

            telemetry.addData("control fps", 1.s / deltaTime)
            telemetry.addData("expansion fps", 1.s / deltaTime2)
            telemetry.addData("distance mm", distance)
            telemetry.update()
        }
    }
}