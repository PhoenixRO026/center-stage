package org.firstinspires.ftc.teamcode.robot

import android.util.Size
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName
import org.firstinspires.ftc.vision.VisionPortalImpl
import org.firstinspires.ftc.vision.VisionProcessor

class VisionPortalImplEx(
    camera: CameraName,
    cameraMonitorViewId: Int,
    autoPauseCameraMonitor: Boolean,
    cameraResolution: Size,
    webcamStreamFormat: StreamFormat,
    processors: Array<VisionProcessor?>
) : VisionPortalImpl(camera, cameraMonitorViewId, autoPauseCameraMonitor, cameraResolution, webcamStreamFormat, processors) {
    fun getCamera() = camera
}