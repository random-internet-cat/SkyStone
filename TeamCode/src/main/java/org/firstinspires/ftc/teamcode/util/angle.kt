package org.firstinspires.ftc.teamcode.util

import kotlin.math.PI

interface RelativeAngle {
    fun toRadians(): RelativeRadians
}

typealias RawAngle = Double

private const val TWO_PI = 2 * PI

private fun radToDeg(value: RawAngle): RawAngle = value * (360.0 / TWO_PI)
private fun degToRad(value: RawAngle): RawAngle = value * (TWO_PI / 360.0)


inline class RelativeRadians(val raw: RawAngle) : RelativeAngle {
    override fun toRadians(): RelativeRadians {
        return this
    }
}

operator fun RelativeRadians.compareTo(other: RelativeRadians) = (this.raw).compareTo(other.raw)

inline class RelativeDegrees(val raw: RawAngle) : RelativeAngle {
    override fun toRadians(): RelativeRadians {
        return RelativeRadians(degToRad(raw))
    }
}

interface AbsoluteAngle {
    fun normalize(): AbsoluteAngle
    fun toRadians(): AbsoluteRadians
}

private fun normalizeWith(value: RawAngle, modulo: RawAngle): RawAngle {
    return (((value % modulo) + modulo) % modulo)
}

inline class AbsoluteRadians(val raw: RawAngle) : AbsoluteAngle {
    override fun normalize(): AbsoluteAngle {
        return AbsoluteRadians(normalizeWith(raw, TWO_PI))
    }

    override fun toRadians(): AbsoluteRadians {
        return this
    }
}

operator fun AbsoluteRadians.compareTo(other: AbsoluteRadians) = (this.raw).compareTo(other.raw)

inline class AbsoluteDegrees(val raw: RawAngle) : AbsoluteAngle {
    override fun normalize(): AbsoluteAngle {
        return AbsoluteDegrees(normalizeWith(raw, 360.0))
    }

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

operator fun AbsoluteAngle.plus(diff: RelativeAngle) = AbsoluteRadians((this.toRadians().raw) + (diff.toRadians().raw))

operator fun RelativeAngle.plus(value: AbsoluteAngle) = value + this

operator fun AbsoluteAngle.minus(diff: RelativeAngle) = AbsoluteRadians((this.toRadians().raw) - (diff.toRadians().raw))

operator fun AbsoluteAngle.minus(other: AbsoluteAngle) = RelativeRadians((this.toRadians().raw) - (other.toRadians().raw))

operator fun RelativeAngle.plus(diff: RelativeAngle) = RelativeRadians(this.toRadians().raw + diff.toRadians().raw)

operator fun RelativeAngle.minus(diff: RelativeAngle) = RelativeRadians(this.toRadians().raw - diff.toRadians().raw)

operator fun RelativeAngle.times(num: Double) = RelativeRadians(this.toRadians().raw * num)

operator fun RelativeAngle.div(num: Double) = RelativeRadians(this.toRadians().raw / num)
