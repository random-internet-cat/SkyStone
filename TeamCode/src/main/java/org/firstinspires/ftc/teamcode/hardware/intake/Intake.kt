package org.firstinspires.ftc.teamcode.hardware.intake

import com.qualcomm.robotcore.hardware.DcMotorSimple

class Intake {
    private val m_motors: List<DcMotorSimple>

    private val DEFAULT_POWER = 1.0

    constructor(vararg motors: DcMotorSimple) {
        m_motors = motors.toList()
    }

    private fun forEachMotor(f: DcMotorSimple.() -> Unit) {
        m_motors.forEach(f)
    }

    private fun power(value: Double) {
        forEachMotor { setPower(value) }
    }

    private fun power(value: Int) = power(value.toDouble())

    fun stop() {
        power(0)
    }

    fun intake(power: Double = DEFAULT_POWER) {
        require(0 <= power && power <= 1)

        power(power)
    }

    fun outtake(power: Double = DEFAULT_POWER) {
        require(0 <= power && power <= 1)

        power(-power)
    }

    inline fun update() {}
}