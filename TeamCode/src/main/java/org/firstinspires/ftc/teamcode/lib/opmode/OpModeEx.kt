package org.firstinspires.ftc.teamcode.lib.opmode

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.teamcode.lib.units.ms
import org.firstinspires.ftc.teamcode.lib.units.s

abstract class OpModeEx : LinearOpMode() {
    private val features: MutableList<Feature> = mutableListOf()
    private val lazyInits: MutableList<OpModeInit> = mutableListOf()
    private var previousTime = 0.ms
    protected var deltaTime = 20.ms
        private set

    protected val fps get() = 1.s / deltaTime

    protected val elapsedTime get() = time.s

    init {
        telemetry = MultipleTelemetry(telemetry, FtcDashboard.getInstance().telemetry)
    }

    fun registerFeature(feature: Feature) {
        features.add(feature)
    }


    fun <T> opModeLazy(constructor: () -> T): OpModeLazy<T> {
        val thing = OpModeLazyImpl {
            constructor()
        }
        lazyInits.add(thing)
        return thing
    }

    private fun updateTime() {
        deltaTime = System.currentTimeMillis().ms - previousTime
        previousTime = System.currentTimeMillis().ms
        TimeOpModeImpl.setDeltaTime(deltaTime)
        TimeOpModeImpl.setElapsedTime(elapsedTime)
    }

    override fun runOpMode() {
        previousTime = System.currentTimeMillis().ms
        updateTime()
        lazyInits.forEach { it.init() }
        features.forEach { it.preInit() }
        initEx()
        features.forEach { it.postInit() }

        while (opModeInInit()) {
            updateTime()
            features.forEach { it.preInitLoop() }
            initLoopEx()
            features.forEach { it.postInitLoop() }
        }

        updateTime()
        features.forEach { it.preStart() }
        startEx()
        features.forEach { it.postStart() }

        while (isStarted && !isStopRequested) {
            updateTime()
            features.forEach { it.preLoop() }
            loopEx()
            features.forEach { it.postLoop() }
        }
        TimeOpModeImpl.setElapsedTime(0.ms)
        TimeOpModeImpl.setDeltaTime(0.ms)
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