package org.firstinspires.ftc.teamcode.util.units

import org.firstinspires.ftc.teamcode.util.TWO_PI

interface AngularVelocity {
    companion object {
        fun zero() = RevolutionsPerSecond.zero()
    }

    fun toRevolutionsPerSecond(): RevolutionsPerSecond
}

typealias RawAngularVelocity = Double

inline class RevolutionsPerSecond(val raw: RawAngularVelocity) : AngularVelocity {
    companion object {
        fun zero() = RevolutionsPerSecond(0.0)
    }

    inline override fun toRevolutionsPerSecond(): RevolutionsPerSecond {
        return this
    }
}

inline fun RevolutionsPerSecond(rot: RevolutionsPerSecond) = rot
fun RevolutionsPerSecond(rot: AngularVelocity) = rot.toRevolutionsPerSecond()

inline operator fun Revolutions.div(seconds: Seconds) = RevolutionsPerSecond(this.raw / seconds.raw)

inline operator fun RevolutionsPerSecond.compareTo(other: RevolutionsPerSecond) = (this.raw).compareTo(other.raw)
inline operator fun RevolutionsPerSecond.unaryMinus() = RevolutionsPerSecond(-this.raw)
inline operator fun RevolutionsPerSecond.plus(other: RevolutionsPerSecond) = RevolutionsPerSecond(this.raw + other.raw)
inline operator fun RevolutionsPerSecond.minus(other: RevolutionsPerSecond) = RevolutionsPerSecond(this.raw - other.raw)
inline operator fun RevolutionsPerSecond.times(num: Double) = RevolutionsPerSecond(this.raw * num)
inline operator fun RevolutionsPerSecond.times(seconds: Seconds) = Revolutions(this.raw * seconds.raw)
inline operator fun RevolutionsPerSecond.div(other: RevolutionsPerSecond) = this.raw / other.raw
inline operator fun RevolutionsPerSecond.div(num: Double) = RevolutionsPerSecond(this.raw / num)

inline operator fun Seconds.times(rot: RevolutionsPerSecond) = rot * this
inline operator fun Double.times(rot: RevolutionsPerSecond) = rot * this

inline class RevolutionsPerMinute(val raw: RawAngularVelocity) : AngularVelocity {
    inline override fun toRevolutionsPerSecond(): RevolutionsPerSecond {
        return RevolutionsPerSecond(raw / SECONDS_PER_MINUTE)
    }
}

inline fun RevolutionsPerMinute(rot: RevolutionsPerSecond) = RevolutionsPerMinute(rot.raw * SECONDS_PER_MINUTE)
inline fun RevolutionsPerMinute(rot: RevolutionsPerMinute) = rot
fun RevolutionsPerMinute(rot: AngularVelocity) = rot

inline operator fun Revolutions.div(minutes: Minutes) = RevolutionsPerMinute(this.raw / minutes.raw)

data class RevolutionsPerTime(val revolutions: Revolutions, val time: Duration) : AngularVelocity {
    inline override fun toRevolutionsPerSecond(): RevolutionsPerSecond {
        return revolutions / time.toSeconds()
    }
}

operator fun Revolutions.div(time: Duration) = RevolutionsPerTime(this, time)

inline class DegreesPerSecond(val raw: RawAngularVelocity) : AngularVelocity {
    override fun toRevolutionsPerSecond(): RevolutionsPerSecond {
        return RevolutionsPerSecond(raw / 360)
    }
}

inline fun DegreesPerSecond(rot: RevolutionsPerSecond) = DegreesPerSecond(rot.raw * 360)
inline fun DegreesPerSecond(raw: Int) = DegreesPerSecond(raw.toDouble())
fun DegreesPerSecond(rot: AngularVelocity) = DegreesPerSecond(rot.toRevolutionsPerSecond())

inline operator fun Degrees.div(time: Seconds) = DegreesPerSecond(this.raw / time.raw)
operator fun Degrees.div(time: Duration) = this / (time.toSeconds())

inline class RadiansPerSecond(val raw: RawAngularVelocity) : AngularVelocity {
    override fun toRevolutionsPerSecond(): RevolutionsPerSecond {
        return RevolutionsPerSecond(raw / TWO_PI)
    }
}

inline fun RadiansPerSecond(rot: RevolutionsPerSecond) = RadiansPerSecond(rot.raw * TWO_PI)
inline fun RadiansPerSecond(raw: Int) = RadiansPerSecond(raw.toDouble())
fun RadiansPerSecond(rot: AngularVelocity) = RadiansPerSecond(rot.toRevolutionsPerSecond())

inline operator fun Radians.div(time: Seconds) = RadiansPerSecond(this.raw / time.raw)
operator fun Radians.div(time: Duration) = this/  (time.toSeconds())

inline operator fun Radians.div(vel: RevolutionsPerSecond) = Seconds(this.raw / vel.raw)
operator fun Radians.div(vel: AngularVelocity) = this / vel.toRevolutionsPerSecond()

operator fun AngularVelocity.compareTo(other: AngularVelocity) = (this.toRevolutionsPerSecond()).compareTo(other.toRevolutionsPerSecond())
operator fun AngularVelocity.unaryMinus() = -(this.toRevolutionsPerSecond())
operator fun AngularVelocity.plus(other: AngularVelocity) = this.toRevolutionsPerSecond() + other.toRevolutionsPerSecond()
operator fun AngularVelocity.minus(other: AngularVelocity) = this.toRevolutionsPerSecond() - other.toRevolutionsPerSecond()
operator fun AngularVelocity.times(num: Double) = this.toRevolutionsPerSecond() * num
operator fun AngularVelocity.times(time: Duration) = this.toRevolutionsPerSecond() * time.toSeconds()
operator fun AngularVelocity.div(other: AngularVelocity) = this.toRevolutionsPerSecond() / other.toRevolutionsPerSecond()
operator fun AngularVelocity.div(num: Double) = this.toRevolutionsPerSecond() / num

operator fun Duration.times(rot: AngularVelocity) = this.toSeconds() * rot.toRevolutionsPerSecond()
operator fun Double.times(rot: AngularVelocity) = this * rot.toRevolutionsPerSecond()

operator fun Angle.div(time: Duration) = (this.toRadians()) / time
operator fun Angle.div(vel: AngularVelocity) = (this.toRadians()) / vel