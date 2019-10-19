package org.firstinspires.ftc.teamcode.util.units

import kotlin.math.abs
import kotlin.math.absoluteValue

interface Distance {
    companion object {
        fun zero() = Meters.zero()
    }

    fun toMeters(): Meters
}

typealias RawDistance = Double

private fun Number.toRawDistance(): RawDistance = this.toDouble()
private fun Int.toRawDistance(): RawDistance = this.toDouble()
private fun Double.toRawDistance(): RawDistance = this.toDouble()

const val INCHES_PER_FOOT = 12
const val FEET_PER_METER = 3.2808399
const val INCHES_PER_METER = INCHES_PER_FOOT * FEET_PER_METER
const val MM_PER_METER = 1000
const val CM_PER_METER = 100

inline class Meters(val raw: RawDistance) : Distance {
    companion object {
        fun zero() = Meters(0.0)
    }

    inline override fun toMeters(): Meters {
        return this
    }
}

inline fun Meters(distance: Meters) = distance
inline fun Meters(distance: Distance) = distance.toMeters()

fun Meters(raw: Int) = Meters(raw.toRawDistance())

inline operator fun Meters.compareTo(other: Meters) = (this.raw).compareTo(other.raw)
inline operator fun Meters.unaryMinus() = Meters(-this.raw)
inline operator fun Meters.plus(other: Meters) = Meters(this.raw + other.raw)
inline operator fun Meters.minus(other: Meters) = this + (-other)
inline operator fun Meters.times(num: Double) = Meters(this.raw * num)
inline operator fun Meters.times(num: Int) = this * (num.toDouble())
inline operator fun Meters.div(other: Meters) = (this.raw) / (other.raw)
inline operator fun Meters.div(other: Double) = Meters(this.raw / other)
inline operator fun Meters.div(other: Int) = this / (other.toDouble())

inline operator fun Double.times(meters: Meters) = meters * this

inline val Meters.absoluteValue get() = Meters(raw.absoluteValue)
inline fun abs(value: Meters) = Meters(abs(value.raw))

inline class Millimeters(val raw: RawDistance) : Distance {
    inline override fun toMeters(): Meters {
        return Meters(raw / MM_PER_METER)
    }
}

inline fun Millimeters(distance: Meters) = Millimeters(distance.raw / MM_PER_METER)
inline fun Millimeters(distance: Millimeters) = distance
fun Millimeters(distance: Distance) = Millimeters(distance.toMeters())

fun Millimeters(raw: Int) = Millimeters(raw.toRawDistance())

inline class Centimeters(val raw: RawDistance) : Distance {
    inline override fun toMeters(): Meters {
        return Meters(raw / CM_PER_METER)
    }
}

inline fun Centimeters(distance: Meters) = Centimeters(distance.raw / CM_PER_METER)
inline fun Centimeters(distance: Centimeters) = distance
fun Centimeters(distance: Distance) = Centimeters(distance.toMeters())

fun Centimeters(raw: Int) = Centimeters(raw.toRawDistance())

inline class Feet(val raw: RawDistance) : Distance {
    inline override fun toMeters(): Meters {
        return Meters(raw / FEET_PER_METER)
    }
}

inline fun Feet(distance: Meters) = Feet(distance.raw / FEET_PER_METER)
inline fun Feet(distance: Feet) = distance
inline fun Feet(distance: Inches) = Feet(distance / INCHES_PER_FOOT)
fun Feet(distance: Distance) = Feet(distance.toMeters())

fun Feet(raw: Int) = Feet(raw.toRawDistance())

inline class Inches(val raw: RawDistance) : Distance {
    inline override fun toMeters(): Meters {
        return Meters(raw / INCHES_PER_METER)
    }
}

inline fun Inches(distance: Meters) = Meters(distance.raw / INCHES_PER_METER)
inline fun Inches(distance: Inches) = distance
inline fun Inches(distance: Feet) = Inches(distance.raw * INCHES_PER_FOOT)
fun Inches(distance: Distance) = Inches(distance.toMeters())

fun Inches(raw: Int) = Inches(raw.toRawDistance())

operator fun Distance.compareTo(other: Distance) = (this.toMeters()).compareTo(other.toMeters())
operator fun Distance.unaryMinus() = -this.toMeters()
operator fun Distance.plus(other: Distance) = this.toMeters() + other.toMeters()
operator fun Distance.minus(other: Distance) = this.toMeters() - other.toMeters()
operator fun Distance.times(other: Double) = this.toMeters() * other
operator fun Distance.times(other: Int) = this.toMeters() * other
operator fun Distance.div(other: Distance) = this.toMeters() / other.toMeters()
operator fun Distance.div(other: Double) = this.toMeters() / other
operator fun Distance.div(other: Int) = this.toMeters() / other

operator fun Double.times(distance: Distance) = this * distance.toMeters()

val Distance.absoluteValue get() = this.toMeters().absoluteValue
fun abs(value: Distance) = abs(value.toMeters())