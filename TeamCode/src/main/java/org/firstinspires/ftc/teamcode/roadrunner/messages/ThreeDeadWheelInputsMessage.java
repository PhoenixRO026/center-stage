package org.firstinspires.ftc.teamcode.roadrunner.messages;

import com.acmerobotics.roadrunner.ftc.PositionVelocityPair;

<<<<<<<< HEAD:TeamCode/src/main/java/org/firstinspires/ftc/teamcode/roadrunner/messages/ThreeDeadWheelEncodersMessage.java
/** @noinspection unused*/
public final class ThreeDeadWheelEncodersMessage {
========
public final class ThreeDeadWheelInputsMessage {
>>>>>>>> remote/master:TeamCode/src/main/java/org/firstinspires/ftc/teamcode/roadrunner/messages/ThreeDeadWheelInputsMessage.java
    public long timestamp;
    public PositionVelocityPair par0;
    public PositionVelocityPair par1;
    public PositionVelocityPair perp;

    public ThreeDeadWheelInputsMessage(PositionVelocityPair par0, PositionVelocityPair par1, PositionVelocityPair perp) {
        this.timestamp = System.nanoTime();
        this.par0 = par0;
        this.par1 = par1;
        this.perp = perp;
    }
}