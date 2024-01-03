package org.firstinspires.ftc.teamcode.evenimente.liga_phoenix


import com.phoenix_ro026.phoenixlib.units.EAST
import com.phoenix_ro026.phoenixlib.units.NORTH
import com.phoenix_ro026.phoenixlib.units.cm
import com.phoenix_ro026.phoenixlib.units.pose2d
import com.phoenix_ro026.phoenixlib.units.splineTo
import com.phoenix_ro026.phoenixlib.units.vector2d
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.Disabled
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive

@Disabled
@Autonomous
class TestAuto : LinearOpMode() {
    val beginPose = pose2d(0.0.cm, 0.0.cm, NORTH)
    override fun runOpMode() {
        val drive = MecanumDrive(hardwareMap, beginPose)
        drive.actionBuilder(beginPose)
            .splineTo(vector2d(30.cm, 30.cm), EAST)

        //runBlocking(drive.actionBuilder(beginPose).build())
    }
}