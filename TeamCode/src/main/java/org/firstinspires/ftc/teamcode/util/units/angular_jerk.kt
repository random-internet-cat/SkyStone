package org.firstinspires.ftc.teamcode.util.units

import org.firstinspires.ftc.teamcode.util.TWO_PI

typealias RawAngularJerk = Double

interface AngularJerk {
    companion object {
        fun zero() = RevolutionsPerSecondCubed.zero()
    }

    fun toRevolutionsPerSecondCubed(): RevolutionsPerSecondCubed
}

inline class RevolutionsPerSecondCubed(val raw: RawAngularJerk) : AngularJerk {
    companion object {
        fun zero() = RevolutionsPerSecondCubed(0.0)
    }

    inline override fun toRevolutionsPerSecondCubed(): RevolutionsPerSecondCubed {
        return this
    }
}

inline fun RevolutionsPerSecondCubed(raw: Int) = RevolutionsPerSecondCubed(raw.toDouble())
inline fun RevolutionsPerSecondCubed(accel: RevolutionsPerSecondCubed) = accel
fun RevolutionsPerSecondCubed(accel: AngularJerk) = accel.toRevolutionsPerSecondCubed()

inline operator fun RevolutionsPerSecondSquared.div(time: Seconds) = RevolutionsPerSecondCubed(this.raw / time.raw)

inline operator fun RevolutionsPerSecondCubed.compareTo(other: RevolutionsPerSecondCubed) = (this.raw).compareTo(other.raw)
inline operator fun RevolutionsPerSecondCubed.unaryMinus() = RevolutionsPerSecondCubed(-this.raw)
inline operator fun RevolutionsPerSecondCubed.plus(other: RevolutionsPerSecondCubed) = RevolutionsPerSecondCubed(this.raw + other.raw)
inline operator fun RevolutionsPerSecondCubed.minus(other: RevolutionsPerSecondCubed) = RevolutionsPerSecondCubed(this.raw - other.raw)
inline operator fun RevolutionsPerSecondCubed.times(time: Seconds) = RevolutionsPerSecondSquared(this.raw * time.raw)
inline operator fun RevolutionsPerSecondCubed.times(num: Double) = RevolutionsPerSecondCubed(this.raw * num)
inline operator fun RevolutionsPerSecondCubed.div(num: Double) = RevolutionsPerSecondCubed(this.raw / num)

inline operator fun Seconds.times(time: RevolutionsPerSecondCubed) = time * this
inline operator fun Double.times(vel: RevolutionsPerSecondCubed) = vel * this
inline operator fun RevolutionsPerSecondSquared.div(vel: RevolutionsPerSecondCubed) = Seconds(this.raw / vel.raw)

data class AngularAccelerationPerSecond(val accel: AngularAcceleration, val time: Seconds) : AngularJerk {
    inline override fun toRevolutionsPerSecondCubed(): RevolutionsPerSecondCubed {
        return accel.toRevolutionsPerSecondSquared() / time
    }
}

operator fun AngularAcceleration.div(time: Seconds) = AngularAccelerationPerSecond(this, time)

data class AngularAccelerationPerTime(val accel: AngularAcceleration, val time: Duration) : AngularJerk {
    inline override fun toRevolutionsPerSecondCubed(): RevolutionsPerSecondCubed {
        return accel.toRevolutionsPerSecondSquared() / time.toSeconds()
    }
}

operator fun AngularAcceleration.div(time: Duration) = AngularAccelerationPerTime(this, time)

inline class RadiansPerSecondCubed(val raw: RawAngularJerk) : AngularJerk {
    inline override fun toRevolutionsPerSecondCubed(): RevolutionsPerSecondCubed {
        return RevolutionsPerSecondCubed(raw / TWO_PI)
    }
}

inline fun RadiansPerSecondCubed(raw: Int) = RadiansPerSecondCubed(raw.toDouble())
inline fun RadiansPerSecondCubed(accel: RevolutionsPerSecondCubed) = RadiansPerSecondCubed(accel.raw * TWO_PI)
inline fun RadiansPerSecondCubed(accel: RadiansPerSecondCubed) = accel
fun RadiansPerSecondCubed(accel: AngularJerk) = RadiansPerSecondCubed(accel.toRevolutionsPerSecondCubed())

operator fun AngularJerk.compareTo(other: AngularJerk) = (this.toRevolutionsPerSecondCubed()).compareTo(other.toRevolutionsPerSecondCubed())
operator fun AngularJerk.unaryMinus() = -this.toRevolutionsPerSecondCubed()
operator fun AngularJerk.plus(other: AngularJerk) = this.toRevolutionsPerSecondCubed() + other.toRevolutionsPerSecondCubed()
operator fun AngularJerk.minus(other: AngularJerk) = this.toRevolutionsPerSecondCubed() - other.toRevolutionsPerSecondCubed()
operator fun AngularJerk.times(time: Duration) = this.toRevolutionsPerSecondCubed() * time.toSeconds()
operator fun AngularJerk.times(num: Double) = this.toRevolutionsPerSecondCubed() * num
operator fun AngularJerk.div(num: Double) = this.toRevolutionsPerSecondCubed() / num

operator fun Duration.times(vel: AngularJerk) = this.toSeconds() * vel.toRevolutionsPerSecondCubed()
operator fun Double.times(vel: AngularJerk) = this * vel.toRevolutionsPerSecondCubed()
operator fun AngularAcceleration.div(vel: AngularJerk) = this.toRevolutionsPerSecondSquared() / vel.toRevolutionsPerSecondCubed()