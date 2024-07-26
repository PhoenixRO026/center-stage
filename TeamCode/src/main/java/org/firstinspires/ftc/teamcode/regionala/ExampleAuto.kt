package org.firstinspires.ftc.teamcode.regionala

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.canvas.Canvas
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.acmerobotics.roadrunner.SequentialAction
import com.outoftheboxrobotics.photoncore.Photon
import com.qualcomm.hardware.lynx.LynxModule
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.Disabled
import com.qualcomm.robotcore.hardware.configuration.LynxConstants
import org.firstinspires.ftc.teamcode.lib.opmode.MultiThreadOpMode
import org.firstinspires.ftc.teamcode.lib.units.Pose
import org.firstinspires.ftc.teamcode.lib.units.SleepAction
import org.firstinspires.ftc.teamcode.lib.units.Time
import org.firstinspires.ftc.teamcode.lib.units.cm
import org.firstinspires.ftc.teamcode.lib.units.deg
import org.firstinspires.ftc.teamcode.lib.units.ms
import org.firstinspires.ftc.teamcode.lib.units.s
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDriveEx
import org.firstinspires.ftc.teamcode.robot.ArmMulti.Companion.armMulti
import org.firstinspires.ftc.teamcode.robot.ClawMulti.Companion.clawMulti
import org.firstinspires.ftc.teamcode.robot.ColorSensorsMulti.Companion.colorSensMulti
import org.firstinspires.ftc.teamcode.robot.Lift
import org.firstinspires.ftc.teamcode.robot.LiftMulti.Companion.liftMulti
import org.firstinspires.ftc.teamcode.robot.hardware.controlHub
import org.firstinspires.ftc.teamcode.robot.hardware.expansionHub

@Disabled
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

        val expansionHub = hardwareMap.expansionHub()

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

        val controlHub = hardwareMap.controlHub()

        controlHub.bulkCachingMode = LynxModule.BulkCachingMode.MANUAL

        val dash = FtcDashboard.getInstance()

        telemetry = MultipleTelemetry(telemetry, dash.telemetry)

        val action = SequentialAction(
            lift.goToPass(),
            arm.goToScore(),
            SleepAction(1.s),
            arm.goToRamp(),
            lift.goToRamp()
        )

        waitForStart()

        val c = Canvas()
        action.preview(c)

        var running = true

        while (isStarted && !isStopRequested && running) {
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
            telemetry.addData("lift busy", lift.isBusy)
            telemetry.addData("arm busy", arm.isBusy)
            telemetry.update()
        }
    }
}