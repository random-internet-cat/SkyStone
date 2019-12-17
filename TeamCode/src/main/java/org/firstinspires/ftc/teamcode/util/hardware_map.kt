package org.firstinspires.ftc.teamcode.util

import com.qualcomm.hardware.bosch.BNO055IMU
import com.qualcomm.robotcore.hardware.*
import org.firstinspires.ftc.teamcode.hardware.imu.InternalIMU
import org.openftc.revextensions2.ExpansionHubEx

inline fun <reified T> HardwareMap.get(key: String): T = get(T::class.java, key) // Guaranteed not to return null

fun HardwareMap.getHub(key: String = "hub") = get<ExpansionHubEx>(key)
fun HardwareMap.getIMU(key: String = "imu") = InternalIMU(get<BNO055IMU>(key))
fun HardwareMap.getMotor(key: String) = get<DcMotor>(key)
fun HardwareMap.getMotorEx(key: String) = get<DcMotorEx>(key)
fun HardwareMap.getCRServo(key: String) = get<CRServo>(key)
fun HardwareMap.getServo(key: String) = get<Servo>(key)