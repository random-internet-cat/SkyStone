package org.firstinspires.ftc.teamcode.util.units

import org.firstinspires.ftc.teamcode.util.TWO_PI
import kotlin.math.abs
import kotlin.math.cos
import kotlin.math.sin
import kotlin.math.tan

interface Angle {
    companion object {
        fun zero() = Radians.zero()
    }

    fun toRadians(): Radians
}

typealias RawAngle = Double

fun radToDeg(value: RawAngle): RawAngle = value * (360.0 / TWO_PI)
fun degToRad(value: RawAngle): RawAngle = value * (TWO_PI / 360.0)


inline class Radians(val raw: RawAngle) : Angle {
    companion object {
        fun zero() = Radians(0)
    }

    inline override fun toRadians(): Radians {
        return this
    }
}

inline fun Radians(angle: Radians) = angle
inline fun Radians(angle: Angle) = angle.toRadians()

inline operator fun Radians.compareTo(other: Radians) = (this.raw).compareTo(other.raw)
inline operator fun Radians.unaryMinus() = Radians(-this.raw)
inline operator fun Radians.plus(other: Radians) = Radians(this.raw + other.raw)
inline operator fun Radians.minus(other: Radians) = this + (-other)
inline operator fun Radians.times(num: Double) = Degrees(this.raw * num)
inline operator fun Radians.div(num: Double) = Degrees(this.raw / num)

inline operator fun Double.times(angle: Radians) = angle * this

inline fun abs(angle: Radians) = Radians(abs(angle.raw))

inline class Degrees(val raw: RawAngle) : Angle {
    inline override fun toRadians(): Radians {
        return Radians(degToRad(raw))
    }
}

inline fun Degrees(angle: Radians) = Degrees(radToDeg(angle.raw))
inline fun Degrees(angle: Degrees) = angle
fun Degrees(angle: Angle) = Degrees(angle.toRadians())

interface AnglePoint {
    companion object {
        fun zero() = RadiansPoint.zero()
    }

    fun toRadians(): RadiansPoint
}

private fun normalizeWith(value: RawAngle, modulo: RawAngle): RawAngle {
    return (((value % modulo) + modulo) % modulo)
}

inline class RadiansPoint(val raw: RawAngle) : AnglePoint {
    companion object {
        fun zero() = RadiansPoint(0)
    }

    inline override fun toRadians(): RadiansPoint {
        return this
    }
}

inline fun RadiansPoint(angle: RadiansPoint) = angle
inline fun RadiansPoint(angle: AnglePoint) = angle.toRadians()

inline operator fun RadiansPoint.compareTo(other: RadiansPoint) = (this.raw).compareTo(other.raw)
inline operator fun RadiansPoint.plus(diff: Radians) = RadiansPoint(this.raw + diff.raw)
inline operator fun Radians.plus(angle: RadiansPoint) = angle + this
inline operator fun RadiansPoint.minus(diff: Radians) = this + (-diff)
inline operator fun RadiansPoint.minus(other: RadiansPoint) = Radians(this.raw - other.raw)

fun RadiansPoint.normalized() = RadiansPoint(normalizeWith(this.raw, TWO_PI))

inline fun sin(angle: RadiansPoint) = sin(angle.raw)
inline fun cos(angle: RadiansPoint) = cos(angle.raw)
inline fun tan(angle: RadiansPoint) = tan(angle.raw)
inline fun cot(angle: RadiansPoint) = 1 / tan(angle.raw)
inline fun sec(angle: RadiansPoint) = 1 / cos(angle.raw)
inline fun csc(angle: RadiansPoint) = 1 / sin(angle.raw)

inline class DegreesPoint(val raw: RawAngle) : AnglePoint {
    inline override fun toRadians(): RadiansPoint {
        return RadiansPoint(degToRad(raw))
    }
}

inline fun DegreesPoint(angle: RadiansPoint) = DegreesPoint(radToDeg(angle.raw))
inline fun DegreesPoint(angle: DegreesPoint) = angle
fun DegreesPoint(angle: AnglePoint) = DegreesPoint(angle.toRadians())

fun RadiansPoint(raw: Float) = RadiansPoint(raw.toDouble())
fun DegreesPoint(raw: Float) = DegreesPoint(raw.toDouble())

fun RadiansPoint(raw: Int) = RadiansPoint(raw.toDouble())
fun DegreesPoint(raw: Int) = DegreesPoint(raw.toDouble())

fun Radians(raw: Float) = Radians(raw.toDouble())
fun Degrees(raw: Float) = Degrees(raw.toDouble())

fun Radians(raw: Int) = Radians(raw.toDouble())
fun Degrees(raw: Int) = Degrees(raw.toDouble())

fun AnglePoint.toDegrees() = DegreesPoint(this)
fun Angle.toDegrees() = Degrees(this)

operator fun Angle.compareTo(other: Angle) = (this.toRadians()).compareTo(other.toRadians())
operator fun AnglePoint.compareTo(other: AnglePoint) = (this.toRadians()).compareTo(other.toRadians())
operator fun AnglePoint.plus(diff: Angle) = this.toRadians() + diff.toRadians()
operator fun Angle.plus(value: AnglePoint) = this.toRadians() + value.toRadians()
operator fun AnglePoint.minus(diff: Angle) = this.toRadians() - diff.toRadians()
operator fun AnglePoint.minus(other: AnglePoint) = this.toRadians() - other.toRadians()
operator fun Angle.plus(diff: Angle) = this.toRadians() + diff.toRadians()
operator fun Angle.minus(diff: Angle) = this.toRadians() - diff.toRadians()
operator fun Angle.times(num: Double) = this.toRadians() * num
operator fun Angle.div(num: Double) = this.toRadians() / num

operator fun Double.times(angle: Angle) = this * angle.toRadians()

fun AnglePoint.normalized() = this.toRadians().normalized()

fun abs(angle: Angle) = abs(angle.toRadians())

fun sin(angle: AnglePoint) = sin(angle.toRadians())
fun cos(angle: AnglePoint) = cos(angle.toRadians())
fun tan(angle: AnglePoint) = tan(angle.toRadians())
fun cot(angle: AnglePoint) = cot(angle.toRadians())
fun sec(angle: AnglePoint) = sec(angle.toRadians())
fun csc(angle: AnglePoint) = sec(angle.toRadians())