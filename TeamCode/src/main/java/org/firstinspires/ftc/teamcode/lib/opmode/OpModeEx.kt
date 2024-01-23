package org.firstinspires.ftc.teamcode.lib.opmode

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode

abstract class OpModeEx : LinearOpMode() {
    private val features: MutableList<Feature> = mutableListOf()
    private val lazyInits: MutableList<OpModeInit> = mutableListOf()

    fun registerFeature(feature: Feature) {
        features.add(feature)
    }

    fun <T> opModeLazy(constructor: () -> T): OpModeLazy<T> {
        val thing = OpModeLazyImpl(constructor)
        lazyInits.add(thing)
        return thing
    }

    override fun runOpMode() {
        lazyInits.forEach { it.init() }
        features.forEach { it.preInit() }
        initEx()
        features.forEach { it.postInit() }

        while (opModeInInit()) {
            features.forEach { it.preInitLoop() }
            initLoopEx()
            features.forEach { it.postInitLoop() }
        }

        features.forEach { it.preStart() }
        startEx()
        features.forEach { it.postStart() }

        while (isStarted && !isStopRequested) {
            features.forEach { it.preLoop() }
            loopEx()
            features.forEach { it.postLoop() }
        }

        features.forEach { it.preStop() }
        stopEx()
        features.forEach { it.postStop() }
    }

    abstract fun initEx()

    open fun initLoopEx() {}

    open fun startEx() {}

    abstract fun loopEx()

    open fun stopEx() {}
}