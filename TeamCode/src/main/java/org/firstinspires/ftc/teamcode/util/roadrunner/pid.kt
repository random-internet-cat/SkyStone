package org.firstinspires.ftc.teamcode.util.roadrunner

import com.acmerobotics.roadrunner.control.PIDCoefficients as _RRPIDCoefficients
import com.acmerobotics.roadrunner.control.PIDFController
import com.acmerobotics.roadrunner.util.NanoClock
import com.qualcomm.robotcore.hardware.PIDFCoefficients as _FtcPIDFCoefficients
import com.qualcomm.robotcore.hardware.PIDCoefficients as _FtcPIDCoefficients
import org.firstinspires.ftc.teamcode.util.DcMotorCharacterization

// Generate typealiases, so these can be imported instead of raw types everywhere
typealias RRPIDCoefficients = _RRPIDCoefficients
typealias FtcPIDFCoefficients = _FtcPIDFCoefficients
typealias FtcPIDCoefficients = _FtcPIDCoefficients

data class PIDCoefficients(val kP: Double, val kI: Double, val kD: Double) {
    constructor(kP: Int, kI: Int, kD: Int) : this(kP.toDouble(), kI.toDouble(), kD.toDouble())
    constructor(pidf: FtcPIDFCoefficients) : this(pidf.p, pidf.i, pidf.d)
    constructor(pid: RRPIDCoefficients) : this(pid.kP, pid.kI, pid.kD)
}

fun RRPIDCoefficients(pid: PIDCoefficients) = RRPIDCoefficients(pid.kP, pid.kI, pid.kD)

data class PIDFCoefficients(val kP: Double, val kI: Double, val kD: Double, val kF: Double) {
    constructor(pidf: FtcPIDFCoefficients) : this(pidf.p, pidf.i, pidf.d, pidf.f)
    constructor(pid: RRPIDCoefficients, kF: Double) : this(pid.kP, pid.kI, pid.kD, kF)
}

fun RRPIDCoefficients(pid: PIDFCoefficients) = RRPIDCoefficients(pid.kP, pid.kI, pid.kD)

fun PIDFController(pid: RRPIDCoefficients, characterization: DcMotorCharacterization, kF: (Double) -> Double = { 0.0 }, clock: NanoClock = NanoClock.system()) = PIDFController(pid, characterization.kV, characterization.kA, characterization.kStatic, kF, clock)
fun PIDFController(pid: PIDCoefficients, characterization: DcMotorCharacterization, kF: (Double) -> Double = { 0.0 }, clock: NanoClock = NanoClock.system()) = PIDFController(RRPIDCoefficients(pid), characterization.kV, characterization.kA, characterization.kStatic, kF, clock)
