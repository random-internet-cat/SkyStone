package org.firstinspires.ftc.teamcode.util.roadrunner

import com.acmerobotics.dashboard.canvas.Canvas
import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import com.acmerobotics.roadrunner.path.Path
import org.firstinspires.ftc.teamcode.util.units.Inches
import org.firstinspires.ftc.teamcode.util.units.div
import kotlin.math.ceil

/**
 * Set of helper functions for drawing Road Runner paths and trajectories on dashboard canvases.
 */
object DashboardUtil {
    private val DEFAULT_RESOLUTION = Inches(2.0).roadrunner() // distance units; presumed inches
    private val ROBOT_RADIUS = Inches(9.0).roadrunner() // in

    @JvmOverloads
    fun drawSampledPath(canvas: Canvas, path: Path, resolution: RRDistance = DEFAULT_RESOLUTION) {
        val samples = ceil(RRDistance(path.length()) / resolution).toInt()
        val xPoints = DoubleArray(samples)
        val yPoints = DoubleArray(samples)
        val dx = path.length() / (samples - 1)
        for (i in 0 until samples) {
            val displacement = i * dx
            val (x, y) = path[displacement]
            xPoints[i] = x
            yPoints[i] = y
        }
        canvas.strokePolyline(xPoints, yPoints)
    }

    fun drawRobot(canvas: Canvas, pose: Pose2d) {
        canvas.strokeCircle(pose.x, pose.y, ROBOT_RADIUS.roadrunner().raw)
        val (x, y) = pose.headingVec() * (ROBOT_RADIUS.roadrunner().raw)
        val x1 = pose.x + x / 2
        val y1 = pose.y + y / 2
        val x2 = pose.x + x
        val y2 = pose.y + y
        canvas.strokeLine(x1, y1, x2, y2)
    }
}