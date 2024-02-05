package org.firstinspires.ftc.teamcode.regionala

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.canvas.Canvas
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import org.firstinspires.ftc.teamcode.lib.opmode.MultiThreadOpMode
import org.firstinspires.ftc.teamcode.lib.units.Pose
import org.firstinspires.ftc.teamcode.lib.units.Time
import org.firstinspires.ftc.teamcode.lib.units.cm
import org.firstinspires.ftc.teamcode.lib.units.deg
import org.firstinspires.ftc.teamcode.lib.units.ms
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDriveEx
import org.firstinspires.ftc.teamcode.robot.ArmMulti.Companion.armMulti
import org.firstinspires.ftc.teamcode.robot.ClawMulti.Companion.clawMulti
import org.firstinspires.ftc.teamcode.robot.LiftMulti.Companion.liftMulti

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

    private var sideDeltaTime = 20.ms

    override fun sideRunOpMode() {
        var previousTime = System.currentTimeMillis().ms

        waitForStart()

        while (isStarted && !isStopRequested) {
            val now = System.currentTimeMillis().ms
            sideDeltaTime = now - previousTime
            previousTime = now

            arm.write()
            arm.update(sideDeltaTime)
            claw.write()
            lift.read()
            lift.write()
        }
    }

    override fun mainRunOpMode() {
        var previousTime = System.currentTimeMillis().ms
        var deltaTime: Time

        val dash = FtcDashboard.getInstance()

        telemetry = MultipleTelemetry(telemetry, dash.telemetry)

        val action = drive.actionBuilder(startPose)
            .lineToX(20.cm)
            .build()

        waitForStart()

        val c = Canvas()
        action.preview(c)

        var running = true

        while (isStarted && !isStopRequested) {
            val now = System.currentTimeMillis().ms
            deltaTime = now - previousTime
            previousTime = now

            val p = TelemetryPacket()
            p.fieldOverlay().operations.addAll(c.operations)

            if (running) {
                running = action.run(p)
            }

            dash.sendTelemetryPacket(p)
            telemetry.addData("main delta time", deltaTime)
            telemetry.addData("side delta time", sideDeltaTime)
            telemetry.update()
        }
    }
}