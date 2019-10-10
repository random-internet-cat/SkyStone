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