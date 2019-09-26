package org.firstinspires.ftc.teamcode.util.units

import org.firstinspires.ftc.teamcode.util.TWO_PI

typealias RawAngularJerk = Double

interface AngularJerk {
    fun toRevolutionsPerSecondCubed(): RevolutionsPerSecondCubed
}

inline class RevolutionsPerSecondCubed(val raw: RawAngularJerk) : AngularJerk {
    inline override fun toRevolutionsPerSecondCubed(): RevolutionsPerSecondCubed {
        return this
    }
}

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

data class AngularAccelerationPerTime(val accel: AngularAcceleration, val time: Time) : AngularJerk {
    inline override fun toRevolutionsPerSecondCubed(): RevolutionsPerSecondCubed {
        return accel.toRevolutionsPerSecondSquared() / time.toSeconds()
    }
}

operator fun AngularAcceleration.div(time: Time) = AngularAccelerationPerTime(this, time)

inline class RadiansPerSecondCubed(val raw: RawAngularJerk) : AngularJerk {
    inline override fun toRevolutionsPerSecondCubed(): RevolutionsPerSecondCubed {
        return RevolutionsPerSecondCubed(raw / TWO_PI)
    }
}

inline fun RadiansPerSecondCubed(accel: RevolutionsPerSecondCubed) = RadiansPerSecondCubed(accel.raw * TWO_PI)
inline fun RadiansPerSecondCubed(accel: RadiansPerSecondCubed) = accel
fun RadiansPerSecondCubed(accel: AngularJerk) = RadiansPerSecondCubed(accel.toRevolutionsPerSecondCubed())

operator fun AngularJerk.compareTo(other: AngularJerk) = (this.toRevolutionsPerSecondCubed()).compareTo(other.toRevolutionsPerSecondCubed())
operator fun AngularJerk.unaryMinus() = -this.toRevolutionsPerSecondCubed()
operator fun AngularJerk.plus(other: AngularJerk) = this.toRevolutionsPerSecondCubed() + other.toRevolutionsPerSecondCubed()
operator fun AngularJerk.minus(other: AngularJerk) = this.toRevolutionsPerSecondCubed() - other.toRevolutionsPerSecondCubed()
operator fun AngularJerk.times(time: Time) = this.toRevolutionsPerSecondCubed() * time.toSeconds()
operator fun AngularJerk.times(num: Double) = this.toRevolutionsPerSecondCubed() * num
operator fun AngularJerk.div(num: Double) = this.toRevolutionsPerSecondCubed() / num

operator fun Time.times(vel: AngularJerk) = this.toSeconds() * vel.toRevolutionsPerSecondCubed()
operator fun Double.times(vel: AngularJerk) = this * vel.toRevolutionsPerSecondCubed()
operator fun AngularAcceleration.div(vel: AngularJerk) = this.toRevolutionsPerSecondSquared() / vel.toRevolutionsPerSecondCubed()