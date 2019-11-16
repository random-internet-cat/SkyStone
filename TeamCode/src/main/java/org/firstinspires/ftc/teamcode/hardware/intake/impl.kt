package org.firstinspires.ftc.teamcode.hardware.intake

import com.qualcomm.robotcore.hardware.DcMotor

class PrototypeIntake(motors: List<DcMotor>) : FixedPowerIntake(1.0, motors) {
    constructor(vararg motors: DcMotor) : this(motors.toList())
}

class MarkIIntake(motors: List<DcMotor>) : FixedPowerIntake(0.5, motors) {
    constructor(vararg motors: DcMotor) : this(motors.toList())
}