package org.firstinspires.ftc.teamcode.util.units

typealias RawEncoderTicks = Int

inline class EncoderPosition(val raw: RawEncoderTicks) {
    companion object {
        fun zero() = EncoderPosition(0)
    }
}

inline class EncoderTicks(val raw: RawEncoderTicks) {
    companion object {
        fun zero() = EncoderTicks(0)
    }
}

inline operator fun EncoderTicks.unaryMinus() = EncoderTicks(-this.raw)
inline operator fun EncoderTicks.plus(other: EncoderTicks) = EncoderTicks(this.raw + other.raw)
inline operator fun EncoderTicks.minus(other: EncoderTicks) = EncoderTicks(this.raw - other.raw)
inline operator fun EncoderTicks.times(num: Int) = EncoderTicks(this.raw * num)
inline operator fun Int.times(ticks: EncoderTicks) = EncoderTicks(this * ticks.raw)

inline operator fun EncoderPosition.plus(diff: EncoderTicks) = EncoderPosition(this.raw + diff.raw)
inline operator fun EncoderPosition.minus(diff: EncoderTicks) = EncoderPosition(this.raw - diff.raw)

inline operator fun EncoderPosition.minus(other: EncoderPosition) = EncoderTicks(this.raw - other.raw)

typealias RawEncoderTicksPerSecond = Double

inline class EncoderTicksPerSecond(val raw: RawEncoderTicksPerSecond)

inline fun EncoderTicksPerSecond(raw: Int) = EncoderTicksPerSecond(raw.toDouble())

inline operator fun EncoderTicksPerSecond.unaryMinus() = EncoderTicksPerSecond(-this.raw)
inline operator fun EncoderTicksPerSecond.plus(other: EncoderTicksPerSecond) = EncoderTicksPerSecond(this.raw + other.raw)
inline operator fun EncoderTicksPerSecond.minus(other: EncoderTicksPerSecond) = EncoderTicksPerSecond(this.raw - other.raw)
inline operator fun EncoderTicksPerSecond.times(num: Int) = EncoderTicksPerSecond(this.raw * num)
inline operator fun Int.times(ticks: EncoderTicksPerSecond) = EncoderTicksPerSecond(this * ticks.raw)

inline operator fun EncoderTicks.div(time: Seconds) = EncoderTicksPerSecond(this.raw.toDouble() / time.raw)
inline operator fun EncoderTicks.div(time: Time) = this / (time.toSeconds())

inline operator fun EncoderTicksPerSecond.times(time: Seconds) = EncoderTicks((this.raw * time.raw).toInt())
inline operator fun EncoderTicksPerSecond.times(time: Time) = this * time.toSeconds()

inline operator fun Seconds.times(rate: EncoderTicksPerSecond) = rate * this
inline operator fun Time.times(rate: EncoderTicksPerSecond) = (this.toSeconds()) * rate

typealias RawEncoderTicksPerRev = Int

inline class EncoderTicksPerRev(val raw: RawEncoderTicksPerSecond)

inline operator fun EncoderTicksPerRev.unaryMinus() = EncoderTicksPerRev(-this.raw)
inline operator fun EncoderTicksPerRev.plus(other: EncoderTicksPerRev) = EncoderTicksPerRev(this.raw + other.raw)
inline operator fun EncoderTicksPerRev.minus(other: EncoderTicksPerRev) = EncoderTicksPerRev(this.raw - other.raw)
inline operator fun EncoderTicksPerRev.times(num: Int) = EncoderTicksPerRev(this.raw * num)
inline operator fun Int.times(ticks: EncoderTicksPerRev) = EncoderTicksPerRev(this * ticks.raw)

inline operator fun EncoderTicks.div(rev: Revolutions) = EncoderTicksPerRev((this.raw.toDouble() / rev.raw.toDouble().toInt()))

inline operator fun EncoderTicksPerRev.times(rev: Revolutions) = EncoderTicks((this.raw * rev.raw).toInt())
inline operator fun Revolutions.times(ticks: EncoderTicksPerRev) = ticks * this

inline operator fun EncoderTicks.div(ticksPerRev: EncoderTicksPerRev) = Revolutions(this.raw.toDouble() / ticksPerRev.raw.toDouble())