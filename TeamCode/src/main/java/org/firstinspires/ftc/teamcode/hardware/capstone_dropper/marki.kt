package org.firstinspires.ftc.teamcode.hardware.capstone_dropper

import com.acmerobotics.dashboard.config.Config
import com.qualcomm.robotcore.hardware.Servo

@Config
class MarkICapstoneDropper(dropper: Servo) : BaseCapstoneDropper(dropper, object : CapstoneDropperConfig {
    override val inTheWayPosition: CapstoneDropperPosition get() = IN_THE_WAY_POSITION
    override val outOfTheWayPosition: CapstoneDropperPosition get() = OUT_OF_THE_WAY_POSITION
}) {
    companion object {
        @JvmField
        public var IN_THE_WAY_POSITION: Double = 0.0

        @JvmField
        public var OUT_OF_THE_WAY_POSITION: Double = 0.36
    }
}