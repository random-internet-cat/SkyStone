package org.firstinspires.ftc.teamcode.util

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.PIDFCoefficients

fun DcMotor.resetEncoder() {
    this.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER)
    this.setMode(DcMotor.RunMode.RUN_USING_ENCODER)
}

fun DcMotorEx.setAllPID(p: Double, i: Double, d: Double) {
    setPID(DcMotor.RunMode.RUN_USING_ENCODER, p, i, d)
    setPID(DcMotor.RunMode.RUN_TO_POSITION, p, i, d)
}

fun DcMotorEx.setPID(runmode: DcMotor.RunMode, p: Double, i: Double, d: Double) {
    val f = getPIDFCoefficients(runmode).f
    setPIDFCoefficients(runmode, PIDFCoefficients(p, i, d, f))
}