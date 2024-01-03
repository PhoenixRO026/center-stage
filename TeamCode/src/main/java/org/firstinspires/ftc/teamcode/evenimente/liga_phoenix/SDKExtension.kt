package org.firstinspires.ftc.teamcode.evenimente.liga_phoenix

import com.qualcomm.robotcore.hardware.HardwareMap

@Suppress("EXTENSION_SHADOWED_BY_MEMBER")
inline fun <reified T> HardwareMap.get(id: String): T = get(T::class.java, id)