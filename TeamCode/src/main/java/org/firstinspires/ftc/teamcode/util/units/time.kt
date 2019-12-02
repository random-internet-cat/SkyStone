package org.firstinspires.ftc.teamcode.util.units

import com.acmerobotics.roadrunner.util.NanoClock

typealias RawTime = Double

private fun Number.toRawTime() = this.toDouble()
private fun Int.toRawTime() = this.toDouble()

interface Duration {
    companion object {
        fun zero() = Seconds.zero()
    }

    fun toSeconds(): Seconds
}

inline class Seconds(val raw: RawTime) : Duration {
    companion object {
        fun zero() = Seconds(0.0)
    }

    inline override fun toSeconds(): Seconds {
        return this
    }
}

inline fun Seconds(time: Seconds) = time
inline fun Seconds(time: Duration) = time.toSeconds()

fun Seconds(raw: Int) = Seconds(raw.toRawTime())

inline operator fun Seconds.compareTo(other: Seconds) = (this.raw).compareTo(other.raw)
inline operator fun Seconds.unaryMinus() = Seconds(-this.raw)
inline operator fun Seconds.plus(other: Seconds) = Seconds(this.raw + other.raw)
inline operator fun Seconds.minus(other: Seconds) = Seconds(this.raw - other.raw)
inline operator fun Seconds.times(num: Double) = Seconds(this.raw * num)
inline operator fun Seconds.div(num: Double) = Seconds(this.raw / num)
inline operator fun Seconds.div(other: Seconds) = this.raw / other.raw

inline operator fun Double.times(seconds: Seconds) = seconds * this

const val SECONDS_PER_MINUTE = 60

inline class Minutes(val raw: RawTime) : Duration {
    inline override fun toSeconds(): Seconds {
        return Seconds(this.raw / SECONDS_PER_MINUTE)
    }
}

inline fun Minutes(time: Seconds) = Minutes(time.raw / SECONDS_PER_MINUTE)
inline fun Minutes(time: Minutes) = time
fun Minutes(time: Duration) = Minutes(time.toSeconds())

fun Minutes(raw: Int) = Minutes(raw.toRawTime())

operator fun Duration.compareTo(other: Duration) = (this.toSeconds()).compareTo(other.toSeconds())
operator fun Duration.unaryMinus() = -this.toSeconds()
operator fun Duration.plus(other: Duration) = this.toSeconds() + other.toSeconds()
operator fun Duration.minus(other: Duration) = this.toSeconds() - other.toSeconds()
operator fun Duration.times(num: Double) = this.toSeconds() * num
operator fun Duration.div(num: Double) = this.toSeconds() / num
operator fun Duration.div(other: Duration) = this.toSeconds() / other.toSeconds()

operator fun Double.times(time: Duration) = this * time.toSeconds()

interface TimePoint<Tag> {
    fun timeSinceEpoch(): Duration
}

inline class TimeSinceEpoch<Tag>(private val timeSinceEpoch: Duration) : TimePoint<Tag> {
    override fun timeSinceEpoch(): Duration = timeSinceEpoch
}

operator fun <Tag> TimePoint<Tag>.minus(other: TimePoint<Tag>) = this.timeSinceEpoch() - other.timeSinceEpoch()
operator fun <Tag> TimePoint<Tag>.plus(other: Duration) = TimeSinceEpoch<Tag>(this.timeSinceEpoch() + other)

fun <Tag> epochTime() = TimeSinceEpoch<Tag>(Duration.zero())

object SystemTimeTag
typealias SystemTime = TimePoint<SystemTimeTag>

fun systemTimeEpoch(): SystemTime = epochTime()
fun currentSystemTime(): SystemTime = TimeSinceEpoch(Seconds(System.nanoTime() / 1e9))

interface Clock<Tag> {
    fun currentTime(): TimePoint<Tag>
}

typealias SystemClock = Clock<SystemTimeTag>

object DefaultSystemClock : SystemClock {
    override fun currentTime(): SystemTime = currentSystemTime()
}

fun SystemClock.roadrunner(): NanoClock = object : NanoClock() {
    override fun seconds(): Double {
        return Seconds(this@roadrunner.currentTime().timeSinceEpoch()).raw
    }
}