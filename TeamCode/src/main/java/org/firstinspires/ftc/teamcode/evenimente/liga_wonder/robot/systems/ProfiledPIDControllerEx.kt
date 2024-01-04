package org.firstinspires.ftc.teamcode.evenimente.liga_wonder.robot.systems

import com.arcrobotics.ftclib.controller.wpilibcontroller.ProfiledPIDController
import com.arcrobotics.ftclib.trajectory.TrapezoidProfile

class ProfiledPIDControllerEx(
    Kp: Double, Ki: Double, Kd: Double, constraints: TrapezoidProfile.Constraints
) : ProfiledPIDController(Kp, Ki, Kd, constraints) {
    override fun getPeriod(): Double {
        return super.getPeriod()
    }
}