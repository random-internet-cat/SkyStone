package org.firstinspires.ftc.teamcode.util

import com.qualcomm.robotcore.hardware.*

fun HardwareMap.getMotor(key: String): DcMotor = get(DcMotor::class.java, key)
fun HardwareMap.getMotorEx(key: String): DcMotorEx = get(DcMotorEx::class.java, key)
fun HardwareMap.getCRServo(key: String): CRServo = get(CRServo::class.java, key)
fun HardwareMap.getServo(key: String): Servo = get(Servo::class.java, key)