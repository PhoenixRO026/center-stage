package org.firstinspires.ftc.teamcode.lib.opmode

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.teamcode.lib.units.ms
import org.firstinspires.ftc.teamcode.lib.units.s

abstract class OpModeEx : LinearOpMode() {
    private val features: MutableList<Feature> = mutableListOf()
    private val lazyInits: MutableList<OpModeInit> = mutableListOf()
    private var previousTime = 0.ms
    var deltaTime = 20.ms
        private set

    val elapsedTime get() = time.s

    fun registerFeature(feature: Feature) {
        features.add(feature)
    }

    fun <T> opModeLazy(constructor: () -> T): OpModeLazy<T> {
        val thing = OpModeLazyImpl {
            val feature = constructor()
            if (feature is Feature) {
                registerFeature(feature)
            }
            if (feature is TimeOpModeImpl) {
                feature.setElapsedTimeProvider(::elapsedTime)
                feature.setDeltaTimeProvider(::deltaTime)
            }
            feature
        }
        lazyInits.add(thing)
        return thing
    }

    private fun updateDeltaTime() {
        deltaTime = System.currentTimeMillis().ms - previousTime
        previousTime = System.currentTimeMillis().ms
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