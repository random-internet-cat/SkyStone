package org.firstinspires.ftc.teamcode.hardware.intake

import com.qualcomm.robotcore.hardware.DcMotor

class MarkIIntake(motors: List<DcMotor>) : FixedPowerIntake(0.5, motors) {
    constructor(vararg motors: DcMotor) : this(motors.toList())
}