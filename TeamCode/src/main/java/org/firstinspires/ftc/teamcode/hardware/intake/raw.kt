package org.firstinspires.ftc.teamcode.hardware.intake

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorSimple

class RawIntake {
    private val m_motors: List<DcMotorSimple>

    private val DEFAULT_POWER = 1.0

    constructor(motors: List<DcMotorSimple>) {
        m_motors = motors.toList() // Defensive copy
    }

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

    fun update() {}
}

open class FixedPowerIntake {
    private val motorPower: Double
    private val raw: RawIntake

    constructor(motorPower: Double, motors: List<DcMotor>) {
        this.motorPower = motorPower
        this.raw = RawIntake(motors)
    }

    fun intake() = raw.intake(power = motorPower)
    fun outtake() = raw.outtake(power = motorPower)
    fun stop() = raw.stop()

    open fun update() {}
}