package org.firstinspires.ftc.teamcode.hardware.intake

import com.qualcomm.robotcore.hardware.CRServo
import com.qualcomm.robotcore.hardware.DcMotor

class MarkIIntake {
    private val mainMotors: List<DcMotor>
    private val servos: List<CRServo>

    constructor(mainMotors: List<DcMotor>, servos: List<CRServo>) {
        // Defensive copies
        this.mainMotors = mainMotors.toList()
        this.servos = servos.toList()
    }

    private fun powerMotors(newPower: Double) {
        mainMotors.forEach { it.power = newPower }
    }

    private fun powerServos(newPower: Double) {
        servos.forEach { it.power = newPower }
    }

    fun intake() {
        powerMotors(MOTOR_POWER)
        powerServos(SERVO_POWER)
    }

    fun outtake() {
        powerMotors(-MOTOR_POWER)
        powerServos(-SERVO_POWER)
    }

    fun stop() {
        powerMotors(0.0)
        powerServos(0.0)
    }

    fun update() {}

    companion object {
        private const val MOTOR_POWER = 0.5
        private const val SERVO_POWER = 0.8
    }
}