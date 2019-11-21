package org.firstinspires.ftc.teamcode.hardware.drive

import org.firstinspires.ftc.teamcode.hardware.imu.InternalIMU
import org.firstinspires.ftc.teamcode.util.units.RadiansPoint

typealias Heading = RadiansPoint

interface HeadingProvider {
    fun currentHeading(): Heading
}

data class IMUHeadingProvider(private val imu: InternalIMU) : HeadingProvider {
    override fun currentHeading(): Heading {
        imu.update()
        return RadiansPoint(imu.heading())
    }
}
