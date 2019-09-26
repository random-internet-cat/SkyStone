package org.firstinspires.ftc.teamcode.hardware.drive.tank

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import org.firstinspires.ftc.teamcode.hardware.InternalIMU
import org.firstinspires.ftc.teamcode.hardware.drive.BaseDriveEx
import kotlin.math.abs
import kotlin.math.max

class TankDrive : BaseDriveEx {
    private val m_imu: InternalIMU

    constructor(imu: InternalIMU, frontLeft: DcMotorEx, frontRight: DcMotorEx, backLeft: DcMotorEx, backRight: DcMotorEx) : super(frontLeft, frontRight, backLeft, backRight) {
        m_imu = imu
    }

    fun drive(linearPower: Double, turnPower: Double) {
        require(-1 <= linearPower && linearPower <= 1)
        require(-1 <= turnPower && turnPower <= 1)

        var left = linearPower - turnPower
        var right = linearPower + turnPower

        if (abs(left) > 1.0 || abs(right) > 1.0) {
            val bigger = max(abs(left), abs(right))
            left /= bigger
            right /= bigger
        }

        leftPower(left)
        rightPower(right)
    }
}