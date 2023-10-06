package org.firstinspires.ftc.teamcode.kickoff.robot

import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.robotcore.external.Telemetry

class Robot @JvmOverloads constructor(
    hardwareMap: HardwareMap,
    private val telemetry: Telemetry? = null
) {
    //@JvmField val arm = Arm(hardwareMap, telemetry)
    @JvmField val drive = Drive(hardwareMap)
    @JvmField val lift = Lift(hardwareMap, telemetry)
    @JvmField val claw = Intake(hardwareMap, telemetry)

    private var oldTime = System.currentTimeMillis()
    private var newTime = System.currentTimeMillis()

    fun update() {
        newTime = System.currentTimeMillis()
        telemetry?.addData("FPS", 1.0 / (newTime - oldTime) * 1000)
        oldTime = newTime
        //arm.update()
        drive.update()
        lift.update()
        claw.update()
    }
}