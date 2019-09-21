package org.firstinspires.ftc.teamcode.hardware

import com.qualcomm.robotcore.hardware.DcMotorSimple

class Intake {
    private val m_motor: DcMotorSimple

    private val DEFAULT_POWER = 1.0

    constructor(intake: DcMotorSimple) {
        m_motor = intake
    }

    fun stop() {
        m_motor.setPower(0.0)
    }

    fun intake(power: Double = DEFAULT_POWER) {
        require(0 <= power && power <= 1)

        m_motor.setPower(power)
    }

    fun outtake(power: Double = DEFAULT_POWER) {
        require(0 <= power && power <= 1)

        m_motor.setPower(-power)
    }
}