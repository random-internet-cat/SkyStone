package org.firstinspires.ftc.teamcode.hardware

import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.roadrunner.geometry.Pose2d
import com.qualcomm.robotcore.hardware.DcMotor
import org.firstinspires.ftc.teamcode.util.roadrunner.RobotPosition
import org.firstinspires.ftc.teamcode.util.roadrunner.roadrunner
import org.firstinspires.ftc.teamcode.util.units.EncoderTicksPerRev
import org.firstinspires.ftc.teamcode.util.units.Millimeters
import org.firstinspires.ftc.teamcode.util.units.times
import kotlin.math.PI

@Config
object MarkIOdomtetryConstants {
    public var PARALLEL_DISTANCE_IN: Double = 14.5
    public var PARALLEL_FORWARD_DISTANCE_IN: Double = 2.5
    public var BACK_DISTANCE_IN: Double = -6.0
    public val ODO_POD_CIRCUMFERENCE = Millimeters(38) * PI
    public const val POD_TICKS_PER_REV = 1440
}

data class OdoPodDesc(val pose: Pose2d, val motor: DcMotor) {
    constructor(pose: RobotPosition, motor: DcMotor) : this(pose.roadrunner(), motor)
}