package org.firstinspires.ftc.teamcode.util

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.teamcode.util.units.Duration
import org.firstinspires.ftc.teamcode.util.units.Seconds

fun LinearOpMode.sleep(duration: Duration) {
    val msRaw = Seconds(duration).raw.toLong() * 1000
    sleep(msRaw)
}