package org.firstinspires.ftc.teamcode.util.units

typealias RawAcceleration = Double

interface Acceleration {
    fun toMetersPerSecondSquared(): MetersPerSecondSquared
}

inline class MetersPerSecondSquared(val raw: RawAcceleration) : Acceleration {
    inline override fun toMetersPerSecondSquared(): MetersPerSecondSquared {
        return this
    }
}

inline fun MetersPerSecondSquared(accel: MetersPerSecondSquared) = accel
fun MetersPerSecondSquared(accel: Acceleration) = accel.toMetersPerSecondSquared()

inline operator fun MetersPerSecond.div(time: Seconds) = MetersPerSecondSquared(this.raw / time.raw)

inline operator fun MetersPerSecondSquared.compareTo(other: MetersPerSecondSquared) = (this.raw).compareTo(other.raw)
inline operator fun MetersPerSecondSquared.unaryMinus() = MetersPerSecondSquared(-this.raw)
inline operator fun MetersPerSecondSquared.plus(other: MetersPerSecondSquared) = MetersPerSecondSquared(this.raw + other.raw)
inline operator fun MetersPerSecondSquared.minus(other: MetersPerSecondSquared) = MetersPerSecondSquared(this.raw - other.raw)
inline operator fun MetersPerSecondSquared.times(time: Seconds) = MetersPerSecond(this.raw * time.raw)
inline operator fun MetersPerSecondSquared.times(num: Double) = MetersPerSecondSquared(this.raw * num)
inline operator fun MetersPerSecondSquared.div(num: Double) = MetersPerSecondSquared(this.raw / num)

inline operator fun Seconds.times(time: MetersPerSecondSquared) = time * this
inline operator fun Double.times(accel: MetersPerSecondSquared) = accel * this

data class VelocityPerSecond(val velocity: Velocity, val time: Seconds) : Acceleration {
    inline override fun toMetersPerSecondSquared(): MetersPerSecondSquared {
        return velocity.toMetersPerSecond() / time
    }
}

operator fun Velocity.div(time: Seconds) = VelocityPerSecond(this, time)

data class VelocityPerTime(val velocity: Velocity, val time: Time) : Acceleration {
    inline override fun toMetersPerSecondSquared(): MetersPerSecondSquared {
        return velocity.toMetersPerSecond() / time.toSeconds()
    }
}

operator fun Velocity.div(time: Time) = VelocityPerTime(this, time)

operator fun Acceleration.compareTo(other: Acceleration) = (this.toMetersPerSecondSquared()).compareTo(other.toMetersPerSecondSquared())
operator fun Acceleration.unaryMinus() = -this.toMetersPerSecondSquared()
operator fun Acceleration.plus(other: Acceleration) = this.toMetersPerSecondSquared() + other.toMetersPerSecondSquared()
operator fun Acceleration.minus(other: Acceleration) = this.toMetersPerSecondSquared() - other.toMetersPerSecondSquared()
operator fun Acceleration.times(time: Time) = this.toMetersPerSecondSquared() * time.toSeconds()
operator fun Acceleration.times(num: Double) = this.toMetersPerSecondSquared() * num
operator fun Acceleration.div(num: Double) = this.toMetersPerSecondSquared() / num

operator fun Time.times(accel: Acceleration) = this.toSeconds() * accel.toMetersPerSecondSquared()
operator fun Double.times(accel: Acceleration) = this * accel.toMetersPerSecondSquared()