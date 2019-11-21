package org.firstinspires.ftc.teamcode.hardware.intake_release

import com.qualcomm.robotcore.hardware.Servo

typealias IntakeReleasePosition = Double

interface IntakeReleaseConfig {
    val servoPosition: IntakeReleasePosition
}

open class BaseIntakeRelease(val servo: Servo, val config: IntakeReleaseConfig) {
    fun release() {
        servo.position = config.servoPosition
    }
}
