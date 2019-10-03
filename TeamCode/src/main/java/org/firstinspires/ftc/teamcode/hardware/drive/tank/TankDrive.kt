package org.firstinspires.ftc.teamcode.hardware.drive.tank

import com.acmerobotics.roadrunner.control.PIDCoefficients
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.PIDFCoefficients
import org.firstinspires.ftc.teamcode.hardware.InternalIMU
import org.firstinspires.ftc.teamcode.hardware.drive.BaseDriveEx
import org.firstinspires.ftc.teamcode.hardware.drive.tank.TankDriveConstants.BASE_CONSTRAINTS
import org.firstinspires.ftc.teamcode.util.roadrunner.PIDCoefficients
import org.firstinspires.ftc.teamcode.util.roadrunner.roadrunner
import org.firstinspires.ftc.teamcode.util.units.Meters
import kotlin.math.abs
import kotlin.math.max

import org.firstinspires.ftc.teamcode.hardware.drive.tank.TankDriveConstants.kV
import org.firstinspires.ftc.teamcode.hardware.drive.tank.TankDriveConstants.kA
import org.firstinspires.ftc.teamcode.hardware.drive.tank.TankDriveConstants.kStatic
import org.firstinspires.ftc.teamcode.hardware.drive.tank.TankDriveConstants.TRACK_WIDTH
import org.firstinspires.ftc.teamcode.util.setPID

class TankDrive : BaseDriveEx {
    private val m_imu: InternalIMU

    constructor(imu: InternalIMU, frontLeft: DcMotorEx, frontRight: DcMotorEx, backLeft: DcMotorEx, backRight: DcMotorEx) : super(frontLeft, frontRight, backLeft, backRight) {
        m_imu = imu
    }

    fun roadrunner(): RRTankDriveBase {
        return object : RRTankDriveBase(kV, kA, kStatic, TRACK_WIDTH, BASE_CONSTRAINTS) {
            override fun getPIDCoefficients(runMode: DcMotor.RunMode): PIDCoefficients {
                return PIDCoefficients(motors()[0].getPIDFCoefficients(runMode))
            }

            override fun setPIDCoefficients(runMode: DcMotor.RunMode, coefficients: PIDCoefficients) {

                forEachMotor {
                    this.setPID(runMode, coefficients)
                }
            }

            override val rawExternalHeading: Double get() = m_imu.heading().toRadians().raw

            private fun positionOf(motor: DcMotor) = TankDriveConstants.encoderTicksToDistance(motor.currentPosition).roadrunner()
            private fun positionOf(motorList: Iterable<DcMotor>) = Meters(motorList.map { positionOf(it).toMeters().raw }.average()).roadrunner()

            override fun getWheelPositions(): List<Double> {
                return listOf(positionOf(leftMotors()).roadrunner().raw, positionOf(rightMotors()).roadrunner().raw)
            }

            override fun setMotorPowers(left: Double, right: Double) {
                leftPower(left)
                rightPower(right)
            }
        }
    }
}