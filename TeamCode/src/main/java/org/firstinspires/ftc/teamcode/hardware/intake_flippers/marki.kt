package org.firstinspires.ftc.teamcode.hardware.intake_flippers

import com.acmerobotics.dashboard.config.Config
import com.qualcomm.robotcore.hardware.CRServo
import com.qualcomm.robotcore.hardware.ServoControllerEx
import java.util.concurrent.Executors
import java.util.concurrent.TimeUnit

private val intakeFlippersExecutor = Executors.newSingleThreadScheduledExecutor()

@Config
class MarkIIntakeFlippers(val servos: List<CRServo>) {
    constructor(vararg servos: CRServo) : this(servos.toList())

    companion object {
        @JvmField
        public var RELEASE_POWER: Double = 0.5

        @JvmField
        public var RELEASE_TIME_MS: Long = 2000
    }

    fun release() {
        servos.forEach { it.power = RELEASE_POWER }
        intakeFlippersExecutor.schedule({
                                            servos.forEach {
                                                it.power = 0.0
                                                val controller = it.controller as ServoControllerEx
                                                controller.setServoPwmDisable(it.portNumber)
                                            }
                                        }, RELEASE_TIME_MS, TimeUnit.MILLISECONDS)
    }

    fun update() {}
}