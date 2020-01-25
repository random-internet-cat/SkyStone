package org.firstinspires.ftc.teamcode.hardware.foundation_mover

import com.acmerobotics.dashboard.config.Config
import com.qualcomm.robotcore.hardware.Servo

@Config
class MarkIFoundationMover(left: Servo, right: Servo) : BaseFoundationMover(left, right, object : FoundationMoverConfig {
    override val grabPosition: FoundationMoverPosition get() = GRAB_POSITION
    override val inTheWayPosition: FoundationMoverPosition get() = IN_THE_WAY_POSITION
    override val outOfTheWayPosition: FoundationMoverPosition get() = OUT_OF_THE_WAY_POSITION
}) {
    companion object {
        @JvmField
        public var GRAB_POSITION: Double = 1.0

        @JvmField
        public var IN_THE_WAY_POSITION: Double = 0.0

        @JvmField
        public var OUT_OF_THE_WAY_POSITION: Double = 0.66
    }
}