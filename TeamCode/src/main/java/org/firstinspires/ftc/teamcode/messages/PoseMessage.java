<<<<<<<< HEAD:TeamCode/src/main/java/org/firstinspires/ftc/teamcode/roadrunner/PoseMessage.java
package org.firstinspires.ftc.teamcode.roadrunner;
========
package org.firstinspires.ftc.teamcode.messages;
>>>>>>>> remote/master:TeamCode/src/main/java/org/firstinspires/ftc/teamcode/messages/PoseMessage.java

import com.acmerobotics.roadrunner.Pose2d;

public final class PoseMessage {
    public long timestamp;
    public double x;
    public double y;
    public double heading;

    public PoseMessage(Pose2d pose) {
        this.timestamp = System.nanoTime();
        this.x = pose.position.x;
        this.y = pose.position.y;
        this.heading = pose.heading.toDouble();
    }
}

