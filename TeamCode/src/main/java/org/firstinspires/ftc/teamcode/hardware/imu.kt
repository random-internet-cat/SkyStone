package org.firstinspires.ftc.teamcode.hardware

import com.qualcomm.hardware.bosch.BNO055IMU
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference
import org.firstinspires.ftc.robotcore.external.navigation.Orientation
import org.firstinspires.ftc.teamcode.util.units.AbsoluteRadians
import org.openftc.revextensions2.ExpansionHubEx

private typealias RawIMU = BNO055IMU

class InternalIMU {
    companion object {
        fun makeOptimized(hub: ExpansionHubEx) = InternalIMU(LynxOptimizedI2cFactory.createLynxEmbeddedImu(hub.standardModule, 0))
    }

    private val imu: RawIMU
    private var angles: Orientation

    public constructor(rawIMU: RawIMU) {
        imu = rawIMU

        val parameters = BNO055IMU.Parameters()
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC
        parameters.loggingEnabled = true
        parameters.loggingTag = "IMU"

        imu.initialize(parameters)

        angles = Orientation()
    }

    public fun update() {
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS)
    }

    public fun heading() = AbsoluteRadians(angles.firstAngle)
    public fun roll() = AbsoluteRadians(angles.secondAngle)
    public fun pitch() = AbsoluteRadians(angles.thirdAngle)
}