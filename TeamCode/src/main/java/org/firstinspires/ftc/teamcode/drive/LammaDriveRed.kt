package org.firstinspires.ftc.teamcode.drive

import com.acmerobotics.roadrunner.Vector2d
import com.outoftheboxrobotics.photoncore.Photon
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.phoenix.phoenixlib.units.deg
import com.phoenix.phoenixlib.units.rotate

@Photon
@TeleOp(group = "Normal")
class LammaDriveRed : LammaDrive() {
    override fun driveFieldCentric(forward: Double, strafe: Double, turn: Double) {
        val newInputVec = Vector2d(forward, strafe).rotate(drive.pose.heading.log() + 90.deg.rad)
        driveRobotCentric(newInputVec.x, newInputVec.y, turn)
    }
}