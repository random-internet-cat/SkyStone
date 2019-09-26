package org.firstinspires.ftc.teamcode.util.roadrunner

import com.acmerobotics.roadrunner.control.PIDCoefficients
import com.qualcomm.robotcore.hardware.PIDFCoefficients

fun PIDCoefficients(pidf: PIDFCoefficients) = PIDCoefficients(pidf.p, pidf.i, pidf.d)