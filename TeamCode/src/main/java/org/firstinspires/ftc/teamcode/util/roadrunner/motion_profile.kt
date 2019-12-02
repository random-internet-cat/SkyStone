package org.firstinspires.ftc.teamcode.util.roadrunner

import com.acmerobotics.roadrunner.profile.MotionProfile
import org.firstinspires.ftc.teamcode.util.units.Duration

operator fun MotionProfile.get(time: RRDuration) = this[time.roadrunner().raw]
operator fun MotionProfile.get(time: Duration) = this[time.roadrunner()]