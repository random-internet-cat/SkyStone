package org.firstinspires.ftc.teamcode.hardware.drive.tank

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import org.firstinspires.ftc.teamcode.hardware.drive.BaseDriveEx
import org.firstinspires.ftc.teamcode.hardware.drive.FourWheelDrivetrainEx
import org.firstinspires.ftc.teamcode.hardware.drive.tank.TankDriveConstants.AXIAL_PID
import org.firstinspires.ftc.teamcode.hardware.drive.tank.TankDriveConstants.BASE_CONSTRAINTS
import org.firstinspires.ftc.teamcode.hardware.drive.tank.TankDriveConstants.CROSS_TRACK_PID
import org.firstinspires.ftc.teamcode.hardware.drive.tank.TankDriveConstants.FEEDFORWARD
import org.firstinspires.ftc.teamcode.hardware.drive.tank.TankDriveConstants.HEADING_PID
import org.firstinspires.ftc.teamcode.util.roadrunner.PIDCoefficients
import org.firstinspires.ftc.teamcode.util.roadrunner.roadrunner
import org.firstinspires.ftc.teamcode.util.units.Meters

import org.firstinspires.ftc.teamcode.hardware.drive.tank.TankDriveConstants.TRACK_WIDTH
import org.firstinspires.ftc.teamcode.hardware.imu.InternalIMU
import org.firstinspires.ftc.teamcode.util.setPID

class TankDrive(private val imu: InternalIMU, frontLeft: DcMotorEx, frontRight: DcMotorEx, backLeft: DcMotorEx, backRight: DcMotorEx) : BaseDriveEx(FourWheelDrivetrainEx(frontLeft, frontRight, backLeft, backRight)) {
    private val roadrunnerValue by lazy {
        object : RRTankDriveBase(TankDrivetrainConfig(trackWidth = TRACK_WIDTH), TankDrivePID(axialPID = AXIAL_PID, headingPID = HEADING_PID, crossTrackPID = CROSS_TRACK_PID), FEEDFORWARD, BASE_CONSTRAINTS) {
            override fun getPIDCoefficients(runMode: DcMotor.RunMode): PIDCoefficients {
                return PIDCoefficients(motors()[0].getPIDFCoefficients(runMode))
            }

            override fun setPIDCoefficients(runMode: DcMotor.RunMode, coefficients: PIDCoefficients) {

                forEachMotor {
                    this.setPID(runMode, coefficients)
                }
            }

            override val rawExternalHeading: Double get() {
                imu.update()
                return imu.heading().toRadians().raw
            }

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

    fun roadrunner() = roadrunnerValue

    fun update() = roadrunner().update()
}