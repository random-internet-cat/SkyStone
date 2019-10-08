package org.firstinspires.ftc.teamcode.util.roadrunner

import com.acmerobotics.roadrunner.control.PIDCoefficients
import com.acmerobotics.roadrunner.control.PIDFController
import com.acmerobotics.roadrunner.util.NanoClock
import com.qualcomm.robotcore.hardware.PIDFCoefficients
import org.firstinspires.ftc.teamcode.util.DcMotorFeedforward

fun PIDCoefficients(pidf: PIDFCoefficients) = PIDCoefficients(pidf.p, pidf.i, pidf.d)

fun PIDFController(pid: PIDCoefficients, feedforward: DcMotorFeedforward, kF: (Double) -> Double = { 0.0 }, clock: NanoClock = NanoClock.system()) = PIDFController(pid, feedforward.kV, feedforward.kA, feedforward.kStatic, kF, clock)