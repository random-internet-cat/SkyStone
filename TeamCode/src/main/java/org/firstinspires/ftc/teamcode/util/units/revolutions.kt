package org.firstinspires.ftc.teamcode.util.units

typealias RawRevolutions = Double

inline class Revolutions(val raw: RawRevolutions)

inline operator fun Revolutions.compareTo(other: Revolutions) = (this.raw).compareTo(other.raw)
inline operator fun Revolutions.unaryMinus() = Revolutions(-this.raw)
inline operator fun Revolutions.plus(other: Revolutions) = Revolutions(this.raw + other.raw)
inline operator fun Revolutions.minus(other: Revolutions) = Revolutions(this.raw - other.raw)
inline operator fun Revolutions.times(num: Double) = Revolutions(this.raw * num)
inline operator fun Revolutions.div(other: Revolutions) = this.raw / other.raw
inline operator fun Revolutions.div(num: Double) = Revolutions(this.raw / num)

inline operator fun Double.times(rev: Revolutions) = rev * this