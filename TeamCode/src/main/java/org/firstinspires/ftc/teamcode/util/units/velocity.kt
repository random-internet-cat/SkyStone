package org.firstinspires.ftc.teamcode.util.units

typealias RawVelocity = Double

interface Velocity {
    companion object {
        fun zero() = MetersPerSecond.zero()
    }

    fun toMetersPerSecond(): MetersPerSecond
}

inline class MetersPerSecond(val raw: RawVelocity) : Velocity {
    companion object {
        fun zero() = MetersPerSecond(0.0)
    }

    inline override fun toMetersPerSecond(): MetersPerSecond {
        return this
    }
}

inline fun MetersPerSecond(vel: MetersPerSecond) = vel
fun MetersPerSecond(vel: Velocity) = vel.toMetersPerSecond()

inline operator fun Meters.div(time: Seconds) = MetersPerSecond(this.raw / time.raw)

inline operator fun MetersPerSecond.compareTo(other: MetersPerSecond) = (this.raw).compareTo(other.raw)
inline operator fun MetersPerSecond.unaryMinus() = MetersPerSecond(-this.raw)
inline operator fun MetersPerSecond.plus(other: MetersPerSecond) = MetersPerSecond(this.raw + other.raw)
inline operator fun MetersPerSecond.minus(other: MetersPerSecond) = MetersPerSecond(this.raw - other.raw)
inline operator fun MetersPerSecond.times(time: Seconds) = Meters(this.raw * time.raw)
inline operator fun MetersPerSecond.times(num: Double) = MetersPerSecond(this.raw * num)
inline operator fun MetersPerSecond.div(num: Double) = MetersPerSecond(this.raw / num)

inline operator fun Seconds.times(time: MetersPerSecond) = time * this
inline operator fun Double.times(vel: MetersPerSecond) = vel * this
inline operator fun Meters.div(vel: MetersPerSecond) = Seconds(this.raw / vel.raw)

data class DistancePerSecond(val distance: Distance, val time: Seconds) : Velocity {
    inline override fun toMetersPerSecond(): MetersPerSecond {
        return distance.toMeters() / time
    }
}

operator fun Distance.div(time: Seconds) = DistancePerSecond(this, time)

data class DistancePerTime(val distance: Distance, val time: Duration) : Velocity {
    inline override fun toMetersPerSecond(): MetersPerSecond {
        return distance.toMeters() / time.toSeconds()
    }
}

operator fun Distance.div(time: Duration) = DistancePerTime(this, time)

operator fun Velocity.compareTo(other: Velocity) = (this.toMetersPerSecond()).compareTo(other.toMetersPerSecond())
operator fun Velocity.unaryMinus() = -this.toMetersPerSecond()
operator fun Velocity.plus(other: Velocity) = this.toMetersPerSecond() + other.toMetersPerSecond()
operator fun Velocity.minus(other: Velocity) = this.toMetersPerSecond() - other.toMetersPerSecond()
operator fun Velocity.times(time: Duration) = this.toMetersPerSecond() * time.toSeconds()
operator fun Velocity.times(num: Double) = this.toMetersPerSecond() * num
operator fun Velocity.div(num: Double) = this.toMetersPerSecond() / num

operator fun Duration.times(vel: Velocity) = this.toSeconds() * vel.toMetersPerSecond()
operator fun Double.times(vel: Velocity) = this * vel.toMetersPerSecond()
operator fun Distance.div(vel: Velocity) = this.toMeters() / vel.toMetersPerSecond()