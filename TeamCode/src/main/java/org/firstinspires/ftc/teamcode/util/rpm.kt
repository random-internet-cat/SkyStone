package org.firstinspires.ftc.teamcode.util

interface RevolutionSpeed {
    fun toRevolutionsPerSecond(): RevolutionsPerSecond
}

typealias RawRotationSpeed = Double

inline class RevolutionsPerSecond(val raw: RawRotationSpeed) : RevolutionSpeed {
    inline override fun toRevolutionsPerSecond(): RevolutionsPerSecond {
        return this
    }
}

inline fun RevolutionsPerSecond(rot: RevolutionsPerSecond) = rot
fun RevolutionsPerSecond(rot: RevolutionSpeed) = rot.toRevolutionsPerSecond()

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

inline class RevolutionsPerMinute(val raw: RawRotationSpeed) : RevolutionSpeed {
    inline override fun toRevolutionsPerSecond(): RevolutionsPerSecond {
        return RevolutionsPerSecond(raw / SECONDS_PER_MINUTE)
    }
}

inline fun RevolutionsPerMinute(rot: RevolutionsPerSecond) = RevolutionsPerMinute(rot.raw * SECONDS_PER_MINUTE)
inline fun RevolutionsPerMinute(rot: RevolutionsPerMinute) = rot
fun RevolutionsPerMinute(rot: RevolutionSpeed) = rot

inline operator fun Revolutions.div(minutes: Minutes) = RevolutionsPerMinute(this.raw / minutes.raw)

data class RevolutionsPerTime(val revolutions: Revolutions, val time: Time) : RevolutionSpeed {
    inline override fun toRevolutionsPerSecond(): RevolutionsPerSecond {
        return revolutions / time.toSeconds()
    }
}

operator fun Revolutions.div(time: Time) = RevolutionsPerTime(this, time)

inline class DegreesPerSecond(val raw: RawRevolutionSpeed) : AngularVelocity {
    override fun toRevolutionsPerSecond(): RevolutionsPerSecond {
        return RevolutionsPerSecond(raw / 360)
    }
}

inline fun DegreesPerSecond(rot: RevolutionsPerSecond) = DegreesPerSecond(rot.raw * 360)
inline fun DegreesPerSecond(raw: Int) = DegreesPerSecond(raw.toDouble())
fun DegreesPerSecond(rot: RevolutionSpeed) = DegreesPerSecond(rot.toRevolutionsPerSecond())

inline class RadiansPerSecond(val raw: RawRevolutionSpeed) : AngularVelocity {
    override fun toRevolutionsPerSecond(): RevolutionsPerSecond {
        return RevolutionsPerSecond(raw / TWO_PI)
    }
}

inline fun RadiansPerSecond(rot: RevolutionsPerSecond) = RadiansPerSecond(rot.raw * TWO_PI)
inline fun RadiansPerSecond(raw: Int) = RadiansPerSecond(raw.toDouble())
fun RadiansPerSecond(rot: RevolutionSpeed) = RadiansPerSecond(rot.toRevolutionsPerSecond())

operator fun RevolutionSpeed.compareTo(other: RevolutionSpeed) = (this.toRevolutionsPerSecond()).compareTo(other.toRevolutionsPerSecond())
operator fun RevolutionSpeed.unaryMinus() = -(this.toRevolutionsPerSecond())
operator fun RevolutionSpeed.plus(other: RevolutionSpeed) = this.toRevolutionsPerSecond() + other.toRevolutionsPerSecond()
operator fun RevolutionSpeed.minus(other: RevolutionSpeed) = this.toRevolutionsPerSecond() - other.toRevolutionsPerSecond()
operator fun RevolutionSpeed.times(num: Double) = this.toRevolutionsPerSecond() * num
operator fun RevolutionSpeed.times(time: Time) = this.toRevolutionsPerSecond() * time.toSeconds()
operator fun RevolutionSpeed.div(other: RevolutionSpeed) = this.toRevolutionsPerSecond() / other.toRevolutionsPerSecond()
operator fun RevolutionSpeed.div(num: Double) = this.toRevolutionsPerSecond() / num

operator fun Time.times(rot: RevolutionSpeed) = this.toSeconds() * rot.toRevolutionsPerSecond()
operator fun Double.times(rot: RevolutionSpeed) = this * rot.toRevolutionsPerSecond()