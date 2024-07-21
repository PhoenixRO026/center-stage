package org.firstinspires.ftc.teamcode.auto.cri

import com.phoenix.phoenixlib.units.tile
import com.qualcomm.robotcore.eventloop.opmode.Autonomous

@Autonomous(preselectTeleOp = "LammaDriveRed", group = "CRI")
class CRIRedLeftCycleNoPark : CRIRedLeftCycle() {
    override val parkY = 0.tile
}