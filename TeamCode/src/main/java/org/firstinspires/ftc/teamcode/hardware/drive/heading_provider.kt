package org.firstinspires.ftc.teamcode.hardware.drive

import org.firstinspires.ftc.teamcode.hardware.imu.InternalIMU
import org.firstinspires.ftc.teamcode.util.TWO_PI
import org.firstinspires.ftc.teamcode.util.units.Radians
import org.firstinspires.ftc.teamcode.util.units.RadiansPoint
import org.firstinspires.ftc.teamcode.util.units.minus

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

data class InvertedHeadingProvider(private val base: HeadingProvider) : HeadingProvider {
    override fun currentHeading(): Heading {
        return RadiansPoint(TWO_PI - RadiansPoint(base.currentHeading()).raw)
    }
}