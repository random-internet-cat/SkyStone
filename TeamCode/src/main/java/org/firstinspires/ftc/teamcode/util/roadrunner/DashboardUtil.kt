package org.firstinspires.ftc.teamcode.util.roadrunner

import com.acmerobotics.dashboard.canvas.Canvas
import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import com.acmerobotics.roadrunner.path.Path
import org.firstinspires.ftc.teamcode.util.units.Distance
import org.firstinspires.ftc.teamcode.util.units.Inches
import org.firstinspires.ftc.teamcode.util.units.div
import kotlin.math.ceil

/**
 * Set of helper functions for drawing Road Runner paths and trajectories on dashboard canvases.
 */
object DashboardUtil {
    private val DEFAULT_RESOLUTION = Inches(2.0)
    private val ROBOT_RADIUS = Inches(9.0)

    @JvmOverloads
    fun drawSampledPath(canvas: Canvas, path: Path, resolution: Distance = DEFAULT_RESOLUTION) {
        val samples = ceil(RRDistance(path.length()) / resolution).toInt()
        val xPoints = DoubleArray(samples)
        val yPoints = DoubleArray(samples)
        val dx = path.length() / (samples - 1)
        for (i in 0 until samples) {
            val displacement = i * dx
            val (x, y) = path[displacement]
            xPoints[i] = Inches(RRDistance(x)).raw
            yPoints[i] = Inches(RRDistance(y)).raw
        }
        canvas.strokePolyline(xPoints, yPoints)
    }

    fun drawRobot(canvas: Canvas, rawPose: Pose2d) {
        val poseX = Inches(RRDistance(rawPose.x)).raw
        val poseY = Inches(RRDistance(rawPose.y)).raw
        val robotRadius = Inches(ROBOT_RADIUS).raw

        canvas.strokeCircle(poseX, poseY, robotRadius)
        val (x, y) = rawPose.headingVec() * (robotRadius)
        val x1 = poseX + x / 2
        val y1 = poseY + y / 2
        val x2 = poseX + x
        val y2 = poseY + y
        canvas.strokeLine(x1, y1, x2, y2)
    }
}