package org.firstinspires.ftc.teamcode.util

import com.qualcomm.robotcore.hardware.DcMotor

fun DcMotor.resetEncoder() {
    this.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER)
    this.setMode(DcMotor.RunMode.RUN_USING_ENCODER)
}