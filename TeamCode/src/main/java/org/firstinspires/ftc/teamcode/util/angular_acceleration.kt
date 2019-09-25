package org.firstinspires.ftc.teamcode.util

typealias RawAngularAcceleration = Double

interface AngularAcceleration {
    fun toRevolutionsPerSecondSquared(): RevolutionsPerSecondSquared
}

inline class RevolutionsPerSecondSquared(val raw: RawAngularAcceleration) : AngularAcceleration {
    inline override fun toRevolutionsPerSecondSquared(): RevolutionsPerSecondSquared {
        return this
    }
}

inline fun RevolutionsPerSecondSquared(accel: RevolutionsPerSecondSquared) = accel
fun RevolutionsPerSecondSquared(accel: AngularAcceleration) = accel.toRevolutionsPerSecondSquared()

inline operator fun RevolutionsPerSecond.div(time: Seconds) = RevolutionsPerSecondSquared(this.raw / time.raw)

inline operator fun RevolutionsPerSecondSquared.compareTo(other: RevolutionsPerSecondSquared) = (this.raw).compareTo(other.raw)
inline operator fun RevolutionsPerSecondSquared.unaryMinus() = RevolutionsPerSecondSquared(-this.raw)
inline operator fun RevolutionsPerSecondSquared.plus(other: RevolutionsPerSecondSquared) = RevolutionsPerSecondSquared(this.raw + other.raw)
inline operator fun RevolutionsPerSecondSquared.minus(other: RevolutionsPerSecondSquared) = RevolutionsPerSecondSquared(this.raw - other.raw)
inline operator fun RevolutionsPerSecondSquared.times(time: Seconds) = RevolutionsPerSecond(this.raw * time.raw)
inline operator fun RevolutionsPerSecondSquared.times(num: Double) = RevolutionsPerSecondSquared(this.raw * num)
inline operator fun RevolutionsPerSecondSquared.div(num: Double) = RevolutionsPerSecondSquared(this.raw / num)

inline operator fun Seconds.times(time: RevolutionsPerSecondSquared) = time * this
inline operator fun Double.times(vel: RevolutionsPerSecondSquared) = vel * this
inline operator fun RevolutionsPerSecond.div(vel: RevolutionsPerSecondSquared) = Seconds(this.raw / vel.raw)

data class AngularVelocityPerSecond(val AngularVelocity: AngularVelocity, val time: Seconds) : AngularAcceleration {
    inline override fun toRevolutionsPerSecondSquared(): RevolutionsPerSecondSquared {
        return AngularVelocity.toRevolutionsPerSecond() / time
    }
}

operator fun AngularVelocity.div(time: Seconds) = AngularVelocityPerSecond(this, time)

data class AngularVelocityPerTime(val AngularVelocity: AngularVelocity, val time: Time) : AngularAcceleration {
    inline override fun toRevolutionsPerSecondSquared(): RevolutionsPerSecondSquared {
        return AngularVelocity.toRevolutionsPerSecond() / time.toSeconds()
    }
}

operator fun AngularVelocity.div(time: Time) = AngularVelocityPerTime(this, time)

inline class RadiansPerSecondSquared(val raw: RawAngularAcceleration) : AngularAcceleration {
    inline override fun toRevolutionsPerSecondSquared(): RevolutionsPerSecondSquared {
        return RevolutionsPerSecondSquared(raw / TWO_PI)
    }
}

inline fun RadiansPerSecondSquared(accel: RevolutionsPerSecondSquared) = RadiansPerSecondSquared(accel.raw * TWO_PI)
inline fun RadiansPerSecondSquared(accel: RadiansPerSecondSquared) = accel
fun RadiansPerSecondSquared(accel: AngularAcceleration) = RadiansPerSecondSquared(accel.toRevolutionsPerSecondSquared())

inline class DegreesPerSecondSquared(val raw: RawAngularAcceleration) : AngularAcceleration {
    inline override fun toRevolutionsPerSecondSquared(): RevolutionsPerSecondSquared {
        return RevolutionsPerSecondSquared(raw / 360)
    }
}

inline fun DegreesPerSecondSquared(accel: RevolutionsPerSecondSquared) = DegreesPerSecondSquared(accel.raw * 360)
inline fun DegreesPerSecondSquared(accel: DegreesPerSecondSquared) = accel
fun DegreesPerSecondSquared(accel: AngularAcceleration) = DegreesPerSecondSquared(accel.toRevolutionsPerSecondSquared())

operator fun AngularAcceleration.compareTo(other: AngularAcceleration) = (this.toRevolutionsPerSecondSquared()).compareTo(other.toRevolutionsPerSecondSquared())
operator fun AngularAcceleration.unaryMinus() = -this.toRevolutionsPerSecondSquared()
operator fun AngularAcceleration.plus(other: AngularAcceleration) = this.toRevolutionsPerSecondSquared() + other.toRevolutionsPerSecondSquared()
operator fun AngularAcceleration.minus(other: AngularAcceleration) = this.toRevolutionsPerSecondSquared() - other.toRevolutionsPerSecondSquared()
operator fun AngularAcceleration.times(time: Time) = this.toRevolutionsPerSecondSquared() * time.toSeconds()
operator fun AngularAcceleration.times(num: Double) = this.toRevolutionsPerSecondSquared() * num
operator fun AngularAcceleration.div(num: Double) = this.toRevolutionsPerSecondSquared() / num

operator fun Time.times(vel: AngularAcceleration) = this.toSeconds() * vel.toRevolutionsPerSecondSquared()
operator fun Double.times(vel: AngularAcceleration) = this * vel.toRevolutionsPerSecondSquared()
operator fun AngularVelocity.div(vel: AngularAcceleration) = this.toRevolutionsPerSecond() / vel.toRevolutionsPerSecondSquared()