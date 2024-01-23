package org.firstinspires.ftc.teamcode.lib.opmode

interface Feature {
    fun preInit() {}

    fun postInit() {}

    fun preInitLoop() {}

    fun postInitLoop() {}

    fun preStart() {}

    fun postStart() {}

    fun preLoop() {}

    fun postLoop() {}

    fun preStop() {}

    fun postStop() {}
}