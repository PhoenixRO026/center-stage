package org.firstinspires.ftc.teamcode.evenimente.alba.robot

import com.acmerobotics.roadrunner.Pose2d
import com.acmerobotics.roadrunner.PoseVelocity2d
import com.acmerobotics.roadrunner.Vector2d
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive
import kotlin.math.cos
import kotlin.math.sin

class Drive2(
    hardwareMap: HardwareMap,
    private val telemetry: Telemetry? = null,
    startPose: Pose2d
) {
    val drive = MecanumDrive(hardwareMap, startPose)

    val sniperSpeed = 0.5

    private var speed = 1.0

    var sniperMode = false
        set(value) {
            speed = if (value) {
                sniperSpeed
            } else {
                1.0
            }
            field = value
        }

    val heading: Double
        get() = drive.imu.robotYawPitchRollAngles.getYaw(AngleUnit.RADIANS)

    fun update() {
        drive.updatePoseEstimate()
    }

    fun driveRobotCentric(forward: Double, strafe: Double, turn: Double)
    = drive.setDrivePowers(PoseVelocity2d(
        Vector2d(
            forward * speed,
            -strafe * speed
        ),
        -turn * speed
    ))

    fun driveFieldCentric(forward: Double, strafe: Double, turn: Double) {
        val newInputVec = Vector2d(forward, strafe).rotate(heading)
        driveRobotCentric(newInputVec.x, newInputVec.y, turn)
    }

    fun resetFieldCentric() {
        drive.imu.resetYaw()
    }

    private fun Vector2d.rotate(angle: Double): Vector2d {
        return Vector2d (
            x = x * cos(angle) - y * sin(angle),
            y = x * sin(angle) + y * cos(angle)
        )
    }
}