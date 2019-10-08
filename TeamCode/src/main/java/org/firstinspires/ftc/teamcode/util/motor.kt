package org.firstinspires.ftc.teamcode.util

import com.acmerobotics.roadrunner.control.PIDCoefficients
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.PIDFCoefficients
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType

fun DcMotor.resetEncoder() {
    this.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER)
    this.setMode(DcMotor.RunMode.RUN_USING_ENCODER)
}

fun DcMotorEx.setPID(runmode: DcMotor.RunMode, p: Double, i: Double, d: Double) {
    val f = getPIDFCoefficients(runmode).f
    setPIDFCoefficients(runmode, PIDFCoefficients(p, i, d, f))
}

fun DcMotorEx.setPID(runmode: DcMotor.RunMode, pid: PIDCoefficients) {
    setPID(runmode, pid.kP, pid.kI, pid.kD)
}

fun requireValidPower(power: Double) {
    require(-1 <= power && power <= 1)
}

fun requirePositivePower(power: Double) {
    require(0 <= power && power <= 1)
}

data class BasicTypedMotor<MotorType : DcMotor>(val motor: MotorType, val config: MotorConfigurationType) {
    constructor(motor: MotorType, configType: Class<*>) : this(motor, MotorConfigurationType.getMotorType(configType))
}

typealias TypedMotor = BasicTypedMotor<DcMotor>
typealias TypedMotorEx = BasicTypedMotor<DcMotorEx>

data class DcMotorFeedforward(val kV: Double, val kA: Double, val kStatic: Double)