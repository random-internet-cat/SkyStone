package org.firstinspires.ftc.teamcode.util

import kotlin.math.abs

private const val _ZERO_TOLERANCE = 0.01f
private const val ZERO_TOLERANCE_D = _ZERO_TOLERANCE.toDouble()
private const val ZERO_TOLERANCE_F = _ZERO_TOLERANCE.toFloat()

fun Double.cutoffToZero() = if (abs(this) < ZERO_TOLERANCE_D) 0.0 else this
fun Float.cutoffToZero() = if(abs(this) < ZERO_TOLERANCE_F) 0.0f else this