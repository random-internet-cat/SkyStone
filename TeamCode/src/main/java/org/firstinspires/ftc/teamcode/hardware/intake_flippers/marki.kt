package org.firstinspires.ftc.teamcode.hardware.intake_flippers

import com.acmerobotics.dashboard.config.Config
import com.qualcomm.robotcore.hardware.Servo

@Config
class MarkIIntakeFlippers(val servos: List<Servo>) {
    constructor(vararg servos: Servo) : this(servos.toList())

    companion object {
        @JvmField
        public var RELEASE_POSITION = 0.5
    }

    fun release() {
        servos.forEach { it.position = RELEASE_POSITION }
    }

    fun update() {}
}