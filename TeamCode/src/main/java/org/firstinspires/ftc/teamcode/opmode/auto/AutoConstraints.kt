package org.firstinspires.ftc.teamcode.opmode.auto

import org.firstinspires.ftc.teamcode.util.roadrunner.DriveConstraints
import org.firstinspires.ftc.teamcode.util.units.*

@JvmField
internal val AUTO_FAST_CONSTRAINTS = DriveConstraints(Feet(4) / Seconds(1), Inches(35) / Seconds(1) / Seconds(1), RevolutionsPerSecond(0.5), DegreesPerSecondSquared(180))
