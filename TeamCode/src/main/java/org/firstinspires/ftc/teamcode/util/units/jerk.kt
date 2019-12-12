@file:Suppress("NOTHING_TO_INLINE", "OVERRIDE_BY_INLINE")

package org.firstinspires.ftc.teamcode.util.units

typealias RawJerk = Double

interface Jerk {
    companion object {
        fun zero() = MetersPerSecondCubed.zero()
    }

    fun toMetersPerSecondCubed(): MetersPerSecondCubed
}

inline class MetersPerSecondCubed(val raw: RawJerk) : Jerk {
    companion object {
        fun zero() = MetersPerSecondCubed(0.0)
    }

    inline override fun toMetersPerSecondCubed(): MetersPerSecondCubed {
        return this
    }
}

inline fun MetersPerSecondCubed(accel: MetersPerSecondCubed) = accel
fun MetersPerSecondCubed(accel: Jerk) = accel.toMetersPerSecondCubed()

inline operator fun MetersPerSecondSquared.div(time: Seconds) = MetersPerSecondCubed(this.raw / time.raw)

inline operator fun MetersPerSecondCubed.compareTo(other: MetersPerSecondCubed) = (this.raw).compareTo(other.raw)
inline operator fun MetersPerSecondCubed.unaryMinus() = MetersPerSecondCubed(-this.raw)
inline operator fun MetersPerSecondCubed.plus(other: MetersPerSecondCubed) = MetersPerSecondCubed(this.raw + other.raw)
inline operator fun MetersPerSecondCubed.minus(other: MetersPerSecondCubed) = MetersPerSecondCubed(this.raw - other.raw)
inline operator fun MetersPerSecondCubed.times(time: Seconds) = MetersPerSecondSquared(this.raw * time.raw)
inline operator fun MetersPerSecondCubed.times(num: Double) = MetersPerSecondCubed(this.raw * num)
inline operator fun MetersPerSecondCubed.div(num: Double) = MetersPerSecondCubed(this.raw / num)

inline operator fun Seconds.times(time: MetersPerSecondCubed) = time * this
inline operator fun Double.times(jerk: MetersPerSecondCubed) = jerk * this

data class AccelerationPerSecond(val distance: Acceleration, val time: Seconds) : Jerk {
    inline override fun toMetersPerSecondCubed(): MetersPerSecondCubed {
        return distance.toMetersPerSecondSquared() / time
    }
}

operator fun Acceleration.div(time: Seconds) = AccelerationPerSecond(this, time)

data class AccelerationPerTime(val distance: Acceleration, val time: Duration) : Jerk {
    inline override fun toMetersPerSecondCubed(): MetersPerSecondCubed {
        return distance.toMetersPerSecondSquared() / time.toSeconds()
    }
}

operator fun Acceleration.div(time: Duration) = AccelerationPerTime(this, time)

operator fun Jerk.compareTo(other: Jerk) = (this.toMetersPerSecondCubed()).compareTo(other.toMetersPerSecondCubed())
operator fun Jerk.unaryMinus() = -this.toMetersPerSecondCubed()
operator fun Jerk.plus(other: Jerk) = this.toMetersPerSecondCubed() + other.toMetersPerSecondCubed()
operator fun Jerk.minus(other: Jerk) = this.toMetersPerSecondCubed() - other.toMetersPerSecondCubed()
operator fun Jerk.times(time: Duration) = this.toMetersPerSecondCubed() * time.toSeconds()
operator fun Jerk.times(num: Double) = this.toMetersPerSecondCubed() * num
operator fun Jerk.div(num: Double) = this.toMetersPerSecondCubed() / num

operator fun Duration.times(jerk: Jerk) = this.toSeconds() * jerk.toMetersPerSecondCubed()
operator fun Double.times(jerk: Jerk) = this * jerk.toMetersPerSecondCubed()