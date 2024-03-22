package org.firstinspires.ftc.teamcode.drive

import com.acmerobotics.roadrunner.Vector2d
import com.outoftheboxrobotics.photoncore.Photon
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.lib.units.deg
import org.firstinspires.ftc.teamcode.lib.units.rotate

@Photon
@TeleOp
class LammaDriveBlue : LammaDrive() {
    override fun driveFieldCentric(forward: Double, strafe: Double, turn: Double) {
        val newInputVec = Vector2d(forward, strafe).rotate(drive.pose.heading.log() - 90.deg.rad)
        driveRobotCentric(newInputVec.x, newInputVec.y, turn)
    }
}