package org.firstinspires.ftc.teamcode.hardware.foundation_mover

import com.qualcomm.robotcore.hardware.Servo

typealias FoundationMoverPosition = Double

interface FoundationMoverConfig {
    val grabPosition: FoundationMoverPosition
    val inTheWayPosition: FoundationMoverPosition
    val outOfTheWayPosition: FoundationMoverPosition
}

open class BaseFoundationMover(val left: Servo, val right: Servo, val config: FoundationMoverConfig) {
    fun setLeft(newPosition: FoundationMoverPosition) {
        left.position = newPosition
    }

    fun setRight(newPosition: FoundationMoverPosition) {
        right.position = newPosition
    }

    fun grabLeft() = setLeft(config.grabPosition)
    fun grabRight() = setRight(config.grabPosition)
    fun grabBoth() { grabLeft(); grabRight(); }
    fun grab() = grabBoth()

    fun moveLeftToOutOfTheWay() = setLeft(config.outOfTheWayPosition)
    fun moveRightToOutOfTheWay() = setRight(config.outOfTheWayPosition)
    fun moveBothToOutOfTheWay() { moveLeftToOutOfTheWay(); moveRightToOutOfTheWay(); }

    fun moveLeftToInTheWay() = setLeft(config.inTheWayPosition)
    fun moveRightToInTheWay() = setRight(config.inTheWayPosition)
    fun moveBothToInTheWay() { moveLeftToInTheWay(); moveRightToInTheWay(); }

    fun update() {}
}