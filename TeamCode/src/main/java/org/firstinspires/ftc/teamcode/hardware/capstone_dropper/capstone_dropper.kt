package org.firstinspires.ftc.teamcode.hardware.capstone_dropper

import com.qualcomm.robotcore.hardware.Servo

typealias CapstoneDropperPosition = Double

interface CapstoneDropperConfig {
    val inTheWayPosition: CapstoneDropperPosition
    val outOfTheWayPosition: CapstoneDropperPosition
}

open class BaseCapstoneDropper(val dropper: Servo, val config: CapstoneDropperConfig) {
    fun setPosition(newPosition: CapstoneDropperPosition) {
        dropper.position = newPosition
    }

    fun moveInWay() = setPosition(config.inTheWayPosition)
    fun moveOutOfWay() = setPosition(config.outOfTheWayPosition)

    fun update() {}
}