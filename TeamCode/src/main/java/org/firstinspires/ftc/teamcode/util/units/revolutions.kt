package org.firstinspires.ftc.teamcode.util.units

typealias RawRevolutions = Double

inline class Revolutions(val raw: RawRevolutions) {
    companion object {
        fun zero() = Revolutions(0.0)
    }
}

inline operator fun Revolutions.compareTo(other: Revolutions) = (this.raw).compareTo(other.raw)
inline operator fun Revolutions.unaryMinus() = Revolutions(-this.raw)
inline operator fun Revolutions.plus(other: Revolutions) = Revolutions(this.raw + other.raw)
inline operator fun Revolutions.minus(other: Revolutions) = Revolutions(this.raw - other.raw)
inline operator fun Revolutions.times(num: Double) = Revolutions(this.raw * num)
inline operator fun Revolutions.div(other: Revolutions) = this.raw / other.raw
inline operator fun Revolutions.div(num: Double) = Revolutions(this.raw / num)

inline operator fun Double.times(rev: Revolutions) = rev * this

typealias RawDistancePerRev = Double

interface DistancePerRev {
    companion object {
        fun zero() = MetersPerRev.zero()
    }

    fun toMetersPerRev(): MetersPerRev
}

inline class MetersPerRev(val raw: RawDistancePerRev) : DistancePerRev {
    companion object {
        fun zero() = MetersPerRev(0.0)
    }

    inline override fun toMetersPerRev(): MetersPerRev {
        return this
    }
}

fun MetersPerRev(raw: Int) = MetersPerRev(raw.toDouble())
inline fun MetersPerRev(other: MetersPerRev) = other
fun MetersPerRev(other: DistancePerRev) = other.toMetersPerRev()

inline operator fun MetersPerRev.plus(other: MetersPerRev) = MetersPerRev(this.raw + other.raw)
inline operator fun MetersPerRev.minus(other: MetersPerRev) = MetersPerRev(this.raw - other.raw)
inline operator fun MetersPerRev.times(other: RawDistancePerRev) = MetersPerRev(this.raw * other)
inline operator fun RawDistancePerRev.times(other: MetersPerRev) = MetersPerRev(this * other.raw)

inline operator fun MetersPerRev.times(other: Revolutions) = Meters(this.raw * other.raw)
inline operator fun Revolutions.times(other: MetersPerRev) = Meters(this.raw * other.raw)
inline operator fun MetersPerRev.times(other: RevolutionsPerSecond) = MetersPerSecond(this.raw * other.raw)
inline operator fun RevolutionsPerSecond.times(other: MetersPerRev) = MetersPerSecond(this.raw * other.raw)
inline operator fun Meters.div(other: MetersPerRev) = Revolutions(this.raw / other.raw)
inline operator fun Meters.div(other: Revolutions) = MetersPerRev((this.raw.toDouble()) / (other.raw.toDouble()))

operator fun DistancePerRev.plus(other: DistancePerRev) = this.toMetersPerRev() + other.toMetersPerRev()
operator fun DistancePerRev.minus(other: DistancePerRev) = this.toMetersPerRev() - other.toMetersPerRev()
operator fun DistancePerRev.times(other: RawDistancePerRev) = this.toMetersPerRev() * other
operator fun RawDistancePerRev.times(other: DistancePerRev) = this * other.toMetersPerRev()

operator fun DistancePerRev.times(other: Revolutions) = this.toMetersPerRev() * other
operator fun Revolutions.times(other: DistancePerRev) = this * other.toMetersPerRev()
operator fun DistancePerRev.times(other: AngularVelocity) = this.toMetersPerRev() * other.toRevolutionsPerSecond()
operator fun AngularVelocity.times(other: DistancePerRev) = this.toRevolutionsPerSecond() * other.toMetersPerRev()
operator fun Distance.div(other: DistancePerRev) = this.toMeters() / other.toMetersPerRev()
operator fun Distance.div(other: Revolutions) = this.toMeters() / other