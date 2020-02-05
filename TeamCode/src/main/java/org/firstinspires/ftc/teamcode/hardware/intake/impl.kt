package org.firstinspires.ftc.teamcode.hardware.intake

import com.qualcomm.robotcore.hardware.CRServo
import com.qualcomm.robotcore.hardware.DcMotor

class MarkIIntake {
    private val mainMotors: List<DcMotor>

    constructor(mainMotors: List<DcMotor>) {
        // Defensive copies
        this.mainMotors = mainMotors.toList()
    }

    private fun powerMotors(newPower: Double) {
        mainMotors.forEach { it.power = newPower }
    }

    fun intake() {
        powerMotors(MOTOR_POWER)
    }

    fun outtake() {
        powerMotors(-MOTOR_POWER)
    }

    fun stop() {
        powerMotors(0.0)
    }

    fun update() {}

    companion object {
        private const val MOTOR_POWER = 0.85
        private const val SERVO_POWER = 0.8
    }
}