package org.firstinspires.ftc.teamcode.lib.opmode

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.teamcode.lib.units.ms

abstract class OpModeEx : LinearOpMode() {
    private val features: MutableList<Feature> = mutableListOf()
    private val lazyInits: MutableList<OpModeInit> = mutableListOf()
    private val deltaTimeListeners: MutableList<DeltaTimeListener> = mutableListOf()
    private var previousTime = 0.ms
    var deltaTime = 20.ms
        private set

    fun registerFeature(feature: Feature) {
        features.add(feature)
    }

    fun registerDeltaTimeListener(listener: DeltaTimeListener) {
        deltaTimeListeners.add(listener)
    }

    fun <T> opModeLazy(constructor: () -> T): OpModeLazy<T> {
        val thing = OpModeLazyImpl {
            val feature = constructor()
            if (feature is Feature) {
                registerFeature(feature)
            }
            if (feature is DeltaTimeListener) {
                registerDeltaTimeListener(feature)
            }
            feature
        }
        lazyInits.add(thing)
        return thing
    }

    private fun updateDeltaTime() {
        deltaTime = System.currentTimeMillis().ms - previousTime
        previousTime = System.currentTimeMillis().ms
        deltaTimeListeners.forEach {
            if (it is DeltaTimeOpModeImpl) {
                it.updateDeltaTime(deltaTime)
            }
        }
    }

    override fun runOpMode() {
        previousTime = System.currentTimeMillis().ms
        lazyInits.forEach { it.init() }
        features.forEach { it.preInit() }
        initEx()
        features.forEach { it.postInit() }

        while (opModeInInit()) {
            updateDeltaTime()
            features.forEach { it.preInitLoop() }
            initLoopEx()
            features.forEach { it.postInitLoop() }
        }

        updateDeltaTime()
        features.forEach { it.preStart() }
        startEx()
        features.forEach { it.postStart() }

        while (isStarted && !isStopRequested) {
            updateDeltaTime()
            features.forEach { it.preLoop() }
            loopEx()
            features.forEach { it.postLoop() }
        }

        updateDeltaTime()
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