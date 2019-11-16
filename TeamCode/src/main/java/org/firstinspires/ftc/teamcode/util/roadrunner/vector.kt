package org.firstinspires.ftc.teamcode.util.roadrunner

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import org.firstinspires.ftc.teamcode.util.units.AnglePoint
import org.firstinspires.ftc.teamcode.util.units.Distance

data class PositionVector(val x: RRDistance, val y: RRDistance) {
    constructor(x: Distance, y: Distance) : this(x.roadrunner(), y.roadrunner())
}

fun PositionVector.roadrunner() = Vector2d(x.roadrunner().raw, y.roadrunner().raw)

data class RobotPosition(val x: RRDistance, val y: RRDistance, val heading: RRAnglePoint) {
    constructor(vec: PositionVector, heading: RRAnglePoint) : this(vec.x, vec.y, heading)
    constructor(vec: PositionVector, heading: AnglePoint) : this(vec.x, vec.y, heading.roadrunner())

    constructor(x: Distance, y: Distance, heading: AnglePoint) : this(x.roadrunner(), y.roadrunner(), heading.roadrunner())
}

fun RobotPosition.roadrunner() = Pose2d(x.roadrunner().raw, y.roadrunner().raw, heading.roadrunner().raw)