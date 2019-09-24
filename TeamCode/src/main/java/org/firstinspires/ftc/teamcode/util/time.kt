package org.firstinspires.ftc.teamcode.util

private typealias RawTime = Double

private fun Number.toRawTime() = this.toDouble()
private fun Int.toRawTime() = this.toDouble()

interface Time {
    fun toSeconds(): Seconds
}

inline class Seconds(val raw: RawTime) : Time {
    inline override fun toSeconds(): Seconds {
        return this
    }
}

fun Seconds(raw: Int) = Seconds(raw.toRawTime())

inline operator fun Seconds.compareTo(other: Seconds) = (this.raw).compareTo(other.raw)
inline operator fun Seconds.unaryMinus() = Seconds(-this.raw)
inline operator fun Seconds.plus(other: Seconds) = Seconds(this.raw + other.raw)
inline operator fun Seconds.minus(other: Seconds) = Seconds(this.raw - other.raw)
inline operator fun Seconds.times(num: Double) = Seconds(this.raw * num)
inline operator fun Seconds.div(num: Double) = Seconds(this.raw / num)
inline operator fun Seconds.div(other: Seconds) = this.raw / other.raw

operator fun Time.compareTo(other: Time) = (this.toSeconds()).compareTo(other.toSeconds())
operator fun Time.unaryMinus() = -this.toSeconds()
operator fun Time.plus(other: Time) = this.toSeconds() + other.toSeconds()
operator fun Time.minus(other: Time) = this.toSeconds() - other.toSeconds()
operator fun Time.times(num: Double) = this.toSeconds() * num
operator fun Time.div(num: Double) = this.toSeconds() / num
operator fun Time.div(other: Time) = this.toSeconds() / other.toSeconds()
