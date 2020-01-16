package org.firstinspires.ftc.teamcode.hardware.foundation_mover

import com.qualcomm.robotcore.hardware.Servo

typealias FoundationMoverPosition = Double

interface FoundationMoverConfig {
    val releasePosition: FoundationMoverPosition
    val grabPosition: FoundationMoverPosition
    val collectStonePosition: FoundationMoverPosition
    val stoneAboveGroundPosition: FoundationMoverPosition
}

open class BaseFoundationMover(val servos: List<Servo>, val config: FoundationMoverConfig) {
    private fun setPosition(position: FoundationMoverPosition) {
        servos.forEach { it.position = position }
    }

    fun grab() = setPosition(config.grabPosition)
    fun release() = setPosition(config.releasePosition)
    fun moveToCollectHeight() = setPosition(config.collectStonePosition)
    fun moveStoneAboveGround() = setPosition(config.stoneAboveGroundPosition)

    fun update() {}
}