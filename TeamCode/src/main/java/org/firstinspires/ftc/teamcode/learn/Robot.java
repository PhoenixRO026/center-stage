package org.firstinspires.ftc.teamcode.learn;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Robot {
    DcMotorEx motor;

    public Robot(HardwareMap hardwareMap) {
        motor = hardwareMap.get(DcMotorEx.class, "leftFoward");
    }


}
