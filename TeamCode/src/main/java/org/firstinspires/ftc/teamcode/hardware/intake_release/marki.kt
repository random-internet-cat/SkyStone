package org.firstinspires.ftc.teamcode.hardware.intake_release

import com.acmerobotics.dashboard.config.Config
import com.qualcomm.robotcore.hardware.Servo

@Config
class MarkIIntakeRelease(servo: Servo) : BaseIntakeRelease(servo, object : IntakeReleaseConfig {
    override val servoPosition: IntakeReleasePosition get() = POSITION
}) {
    companion object {
        @JvmField
        public var POSITION: Double = 0.8
    }
}