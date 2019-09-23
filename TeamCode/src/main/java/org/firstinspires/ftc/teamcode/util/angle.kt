package org.firstinspires.ftc.teamcode.util

import kotlin.math.PI
import kotlin.math.abs

interface RelativeAngle {
    companion object {
        fun zero() = RelativeRadians.zero()
    }

    fun toRadians(): RelativeRadians
}

typealias RawAngle = Double

private const val TWO_PI = 2 * PI

private fun radToDeg(value: RawAngle): RawAngle = value * (360.0 / TWO_PI)
private fun degToRad(value: RawAngle): RawAngle = value * (TWO_PI / 360.0)


inline class RelativeRadians(val raw: RawAngle) : RelativeAngle {
    companion object {
        fun zero() = RelativeRadians(0)
    }

    override fun toRadians(): RelativeRadians {
        return this
    }
}

inline operator fun RelativeRadians.compareTo(other: RelativeRadians) = (this.raw).compareTo(other.raw)
inline operator fun RelativeRadians.unaryMinus() = RelativeRadians(-this.raw)
inline operator fun RelativeRadians.plus(other: RelativeRadians) = RelativeRadians(this.raw + other.raw)
inline operator fun RelativeRadians.minus(other: RelativeRadians) = this + (-other)
inline operator fun RelativeRadians.times(num: Double) = RelativeDegrees(this.raw * num)
inline operator fun RelativeRadians.div(num: Double) = RelativeDegrees(this.raw / num)

inline fun abs(angle: RelativeRadians) = RelativeRadians(abs(angle.raw))

inline class RelativeDegrees(val raw: RawAngle) : RelativeAngle {
    override fun toRadians(): RelativeRadians {
        return RelativeRadians(degToRad(raw))
    }
}

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

    override fun toRadians(): AbsoluteRadians {
        return this
    }
}

inline operator fun AbsoluteRadians.compareTo(other: AbsoluteRadians) = (this.raw).compareTo(other.raw)
inline operator fun AbsoluteRadians.plus(diff: RelativeRadians) = AbsoluteRadians(this.raw + diff.raw)
inline operator fun RelativeRadians.plus(angle: AbsoluteRadians) = angle + this
inline operator fun AbsoluteRadians.minus(diff: RelativeRadians) = this + (-diff)
inline operator fun AbsoluteRadians.minus(other: AbsoluteRadians) = RelativeRadians(this.raw - other.raw)

fun AbsoluteRadians.normalized() = AbsoluteRadians(normalizeWith(this.raw, TWO_PI))

inline class AbsoluteDegrees(val raw: RawAngle) : AbsoluteAngle {
    override fun toRadians(): AbsoluteRadians {
        return AbsoluteRadians(degToRad(raw))
    }
}

fun AbsoluteRadians(raw: Int) = AbsoluteRadians(raw.toDouble())
fun AbsoluteDegrees(raw: Int) = AbsoluteDegrees(raw.toDouble())

fun RelativeRadians(raw: Int) = RelativeRadians(raw.toDouble())
fun RelativeDegrees(raw: Int) = RelativeDegrees(raw.toDouble())

fun AbsoluteAngle.toDegrees() = AbsoluteDegrees(radToDeg(toRadians().raw))
fun RelativeAngle.toDegrees() = RelativeDegrees(radToDeg(toRadians().raw))

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

fun AbsoluteAngle.normalized() = this.toRadians().normalized()

fun abs(angle: RelativeAngle) = abs(angle.toRadians())