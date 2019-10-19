package org.firstinspires.ftc.teamcode.util.roadrunner

import com.acmerobotics.roadrunner.profile.MotionProfile
import org.firstinspires.ftc.teamcode.util.units.Time

operator fun MotionProfile.get(time: RRTime) = this[time.roadrunner().raw]
operator fun MotionProfile.get(time: Time) = this[time.roadrunner()]