package org.firstinspires.ftc.teamcode.regionala

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.canvas.Canvas
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.outoftheboxrobotics.photoncore.Photon
import com.qualcomm.hardware.lynx.LynxModule
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.hardware.configuration.LynxConstants
import org.firstinspires.ftc.teamcode.lib.opmode.MultiThreadOpMode
import org.firstinspires.ftc.teamcode.lib.units.Distance2d
import org.firstinspires.ftc.teamcode.lib.units.Pose
import org.firstinspires.ftc.teamcode.lib.units.Time
import org.firstinspires.ftc.teamcode.lib.units.cm
import org.firstinspires.ftc.teamcode.lib.units.deg
import org.firstinspires.ftc.teamcode.lib.units.ms
import org.firstinspires.ftc.teamcode.lib.units.s
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDriveEx
import org.firstinspires.ftc.teamcode.robot.ArmMulti.Companion.armMulti
import org.firstinspires.ftc.teamcode.robot.ClawMulti.Companion.clawMulti
import org.firstinspires.ftc.teamcode.robot.ColorSensorsMulti.Companion.colorSensMulti
import org.firstinspires.ftc.teamcode.robot.LiftMulti.Companion.liftMulti

@Autonomous
@Photon
class ExampleAuto : MultiThreadOpMode() {
    private val startPose = Pose(0.cm, 0.cm, 0.deg)

    private val drive by opModeLazy {
        MecanumDriveEx(hardwareMap, startPose)
    }

    private val arm by opModeLazy {
        hardwareMap.armMulti()
    }

    private val claw by opModeLazy {
        hardwareMap.clawMulti()
    }

    private val lift by opModeLazy {
        hardwareMap.liftMulti()
    }

    private val colorSensors by opModeLazy {
        hardwareMap.colorSensMulti()
    }

    private var sideDeltaTime = 20.ms

    override fun sideRunOpMode() {
        var previousTime = System.currentTimeMillis().ms

        val expansionHub = hardwareMap.getAll(LynxModule::class.java).first {
            it.isParent && !LynxConstants.isEmbeddedSerialNumber(it.serialNumber)
        }

        expansionHub.bulkCachingMode = LynxModule.BulkCachingMode.MANUAL

        waitForStart()

        while (isStarted && !isStopRequested) {
            val now = System.currentTimeMillis().ms
            sideDeltaTime = now - previousTime
            previousTime = now

            expansionHub.clearBulkCache()

            arm.write()
            arm.update(sideDeltaTime)
            claw.write()
            lift.read()
            lift.write()
            colorSensors.read()
        }
    }

    override fun mainRunOpMode() {
        var previousTime = System.currentTimeMillis().ms
        var deltaTime: Time

        val controlHub = hardwareMap.getAll(LynxModule::class.java).first {
            it.isParent && LynxConstants.isEmbeddedSerialNumber(it.serialNumber)
        }

        controlHub.bulkCachingMode = LynxModule.BulkCachingMode.MANUAL

        val dash = FtcDashboard.getInstance()

        telemetry = MultipleTelemetry(telemetry, dash.telemetry)

        val action = drive.actionBuilder(startPose)
            .splineTo(Distance2d(20.cm, 20.cm), 90.deg)
            .splineTo(Distance2d(0.cm, 40.cm), 180.deg)
            .build()

        waitForStart()

        val c = Canvas()
        action.preview(c)

        var running = true

        while (isStarted && !isStopRequested) {
            val now = System.currentTimeMillis().ms
            deltaTime = now - previousTime
            previousTime = now

            controlHub.clearBulkCache()

            val p = TelemetryPacket()
            p.fieldOverlay().operations.addAll(c.operations)

            if (running) {
                running = action.run(p)
            }

            dash.sendTelemetryPacket(p)

            telemetry.addData("main delta fps", 1.s / deltaTime)
            telemetry.addData("side delta time", 1.s / sideDeltaTime)
            telemetry.addData("left red", colorSensors.leftColor.red)
            telemetry.addData("left green", colorSensors.leftColor.green)
            telemetry.addData("left blue", colorSensors.leftColor.blue)
            telemetry.addData("left alpha", colorSensors.leftColor.alpha)
            telemetry.addData("left distance mm", colorSensors.leftDistance.mm)
            telemetry.addData("left light", colorSensors.leftLight)
            telemetry.addData("right red", colorSensors.rightColor.red)
            telemetry.addData("right green", colorSensors.rightColor.green)
            telemetry.addData("right blue", colorSensors.rightColor.blue)
            telemetry.addData("right alpha", colorSensors.rightColor.alpha)
            telemetry.addData("right distance mm", colorSensors.rightDistance.mm)
            telemetry.addData("right light", colorSensors.rightLight)
            telemetry.update()
        }
    }
}