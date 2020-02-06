package org.firstinspires.ftc.teamcode.hardware

import com.acmerobotics.dashboard.config.Config

@Config
object MarkIOdomtetryConstants {
    public var PARALLEL_DISTANCE_IN: Double = 14.5
    public var BACK_DISTANCE_IN: Double = -2.5
}

data class OdoPodDesc(val pose: Pose2d, val motor: DcMotor) {
    constructor(pose: RobotPosition, motor: DcMotor) : this(pose.roadrunner(), motor)
}