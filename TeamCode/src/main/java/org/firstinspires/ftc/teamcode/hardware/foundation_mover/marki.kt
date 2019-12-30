package org.firstinspires.ftc.teamcode.hardware.foundation_mover

import com.acmerobotics.dashboard.config.Config
import com.qualcomm.robotcore.hardware.Servo

@Config
class MarkIFoundationMover(servos: List<Servo>) : BaseFoundationMover(servos, object : FoundationMoverConfig {
    override val grabPosition: FoundationMoverPosition get() = GRAB_POSITION
    override val releasePosition: FoundationMoverPosition get() = RELEASE_POSITION
}) {
    constructor(vararg servos: Servo) : this(servos.toList())

    companion object {
        @JvmField
        public var GRAB_POSITION = 0.15

        @JvmField
        public var RELEASE_POSITION = 1.0
    }
}