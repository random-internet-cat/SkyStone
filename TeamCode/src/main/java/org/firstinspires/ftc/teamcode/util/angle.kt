package org.firstinspires.ftc.teamcode.util

import kotlin.math.PI
import kotlin.math.abs
import kotlin.math.cos
import kotlin.math.sin
import kotlin.math.tan

interface RelativeAngle {
    companion object {
        fun zero() = RelativeRadians.zero()
    }

    fun toRadians(): RelativeRadians
}

typealias RawAngle = Double

fun radToDeg(value: RawAngle): RawAngle = value * (360.0 / TWO_PI)
fun degToRad(value: RawAngle): RawAngle = value * (TWO_PI / 360.0)


inline class RelativeRadians(val raw: RawAngle) : RelativeAngle {
    companion object {
        fun zero() = RelativeRadians(0)
    }

    inline override fun toRadians(): RelativeRadians {
        return this
    }
}

inline fun RelativeRadians(angle: RelativeRadians) = angle
inline fun RelativeRadians(angle: RelativeAngle) = angle.toRadians()

inline operator fun RelativeRadians.compareTo(other: RelativeRadians) = (this.raw).compareTo(other.raw)
inline operator fun RelativeRadians.unaryMinus() = RelativeRadians(-this.raw)
inline operator fun RelativeRadians.plus(other: RelativeRadians) = RelativeRadians(this.raw + other.raw)
inline operator fun RelativeRadians.minus(other: RelativeRadians) = this + (-other)
inline operator fun RelativeRadians.times(num: Double) = RelativeDegrees(this.raw * num)
inline operator fun RelativeRadians.div(num: Double) = RelativeDegrees(this.raw / num)

inline operator fun Double.times(angle: RelativeRadians) = angle * this

inline fun abs(angle: RelativeRadians) = RelativeRadians(abs(angle.raw))

inline class RelativeDegrees(val raw: RawAngle) : RelativeAngle {
    inline override fun toRadians(): RelativeRadians {
        return RelativeRadians(degToRad(raw))
    }
}

inline fun RelativeDegrees(angle: RelativeRadians) = RelativeDegrees(radToDeg(angle.raw))
inline fun RelativeDegrees(angle: RelativeDegrees) = angle
fun RelativeDegrees(angle: RelativeAngle) = RelativeDegrees(angle.toRadians())

interface AbsoluteAngle {
    companion object {
        fun zero() = AbsoluteRadians.zero()
    }

    fun toRadians(): AbsoluteRadians
}

private fun normalizeWith(value: RawAngle, modulo: RawAngle): RawAngle {
    return (((value % modulo) + modulo) % modulo)
}

inline class AbsoluteRadians(val raw: RawAngle) : AbsoluteAngle {
    companion object {
        fun zero() = AbsoluteRadians(0)
    }

    inline override fun toRadians(): AbsoluteRadians {
        return this
    }
}

inline fun AbsoluteRadians(angle: AbsoluteRadians) = angle
inline fun AbsoluteRadians(angle: AbsoluteAngle) = angle.toRadians()

inline operator fun AbsoluteRadians.compareTo(other: AbsoluteRadians) = (this.raw).compareTo(other.raw)
inline operator fun AbsoluteRadians.plus(diff: RelativeRadians) = AbsoluteRadians(this.raw + diff.raw)
inline operator fun RelativeRadians.plus(angle: AbsoluteRadians) = angle + this
inline operator fun AbsoluteRadians.minus(diff: RelativeRadians) = this + (-diff)
inline operator fun AbsoluteRadians.minus(other: AbsoluteRadians) = RelativeRadians(this.raw - other.raw)

fun AbsoluteRadians.normalized() = AbsoluteRadians(normalizeWith(this.raw, TWO_PI))

inline fun sin(angle: AbsoluteRadians) = sin(angle.raw)
inline fun cos(angle: AbsoluteRadians) = cos(angle.raw)
inline fun tan(angle: AbsoluteRadians) = tan(angle.raw)
inline fun cot(angle: AbsoluteRadians) = 1 / tan(angle.raw)
inline fun sec(angle: AbsoluteRadians) = 1 / cos(angle.raw)
inline fun csc(angle: AbsoluteRadians) = 1 / sin(angle.raw)

inline class AbsoluteDegrees(val raw: RawAngle) : AbsoluteAngle {
    inline override fun toRadians(): AbsoluteRadians {
        return AbsoluteRadians(degToRad(raw))
    }
}

inline fun AbsoluteDegrees(angle: AbsoluteRadians) = AbsoluteDegrees(radToDeg(angle.raw))
inline fun AbsoluteDegrees(angle: AbsoluteDegrees) = angle
fun AbsoluteDegrees(angle: AbsoluteAngle) = AbsoluteDegrees(angle.toRadians())

fun AbsoluteRadians(raw: Float) = AbsoluteRadians(raw.toDouble())
fun AbsoluteDegrees(raw: Float) = AbsoluteDegrees(raw.toDouble())

fun AbsoluteRadians(raw: Int) = AbsoluteRadians(raw.toDouble())
fun AbsoluteDegrees(raw: Int) = AbsoluteDegrees(raw.toDouble())

fun RelativeRadians(raw: Float) = RelativeRadians(raw.toDouble())
fun RelativeDegrees(raw: Float) = RelativeDegrees(raw.toDouble())

fun RelativeRadians(raw: Int) = RelativeRadians(raw.toDouble())
fun RelativeDegrees(raw: Int) = RelativeDegrees(raw.toDouble())

fun AbsoluteAngle.toDegrees() = AbsoluteDegrees(this)
fun RelativeAngle.toDegrees() = RelativeDegrees(this)

operator fun RelativeAngle.compareTo(other: RelativeAngle) = (this.toRadians()).compareTo(other.toRadians())
operator fun AbsoluteAngle.compareTo(other: AbsoluteAngle) = (this.toRadians()).compareTo(other.toRadians())
operator fun AbsoluteAngle.plus(diff: RelativeAngle) = this.toRadians() + diff.toRadians()
operator fun RelativeAngle.plus(value: AbsoluteAngle) = this.toRadians() + value.toRadians()
operator fun AbsoluteAngle.minus(diff: RelativeAngle) = this.toRadians() - diff.toRadians()
operator fun AbsoluteAngle.minus(other: AbsoluteAngle) = this.toRadians() - other.toRadians()
operator fun RelativeAngle.plus(diff: RelativeAngle) = this.toRadians() + diff.toRadians()
operator fun RelativeAngle.minus(diff: RelativeAngle) = this.toRadians() - diff.toRadians()
operator fun RelativeAngle.times(num: Double) = this.toRadians() * num
operator fun RelativeAngle.div(num: Double) = this.toRadians() / num

operator fun Double.times(angle: RelativeAngle) = this * angle.toRadians()

fun AbsoluteAngle.normalized() = this.toRadians().normalized()

fun abs(angle: RelativeAngle) = abs(angle.toRadians())

fun sin(angle: AbsoluteAngle) = sin(angle.toRadians())
fun cos(angle: AbsoluteAngle) = cos(angle.toRadians())
fun tan(angle: AbsoluteAngle) = tan(angle.toRadians())
fun cot(angle: AbsoluteAngle) = cot(angle.toRadians())
fun sec(angle: AbsoluteAngle) = sec(angle.toRadians())
fun csc(angle: AbsoluteAngle) = sec(angle.toRadians())