package org.firstinspires.ftc.teamcode.hardware.foundation_mover

import com.qualcomm.robotcore.hardware.Servo

typealias FoundationMoverPosition = Double

interface FoundationMoverConfig {
    val releasePosition: FoundationMoverPosition
    val grabPosition: FoundationMoverPosition
    val collectStonePosition: FoundationMoverPosition
    val stoneAboveGroundPosition: FoundationMoverPosition
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

    fun releaseLeft() = setLeft(config.releasePosition)
    fun releaseRight() = setRight(config.releasePosition)
    fun releaseBoth() { releaseLeft(); releaseRight(); }
    fun release() = releaseBoth()

    fun moveLeftToCollectHeight() = setLeft(config.collectStonePosition)
    fun moveRightToCollectheight() = setRight(config.collectStonePosition)
    fun moveBothToCollectHeight() { moveLeftToCollectHeight(); moveRightToCollectheight(); }
    fun moveToCollectHeight() = moveBothToCollectHeight()

    fun moveLeftToStoneAboveGround() = setLeft(config.stoneAboveGroundPosition)
    fun moveRightToStoneAboveGround() = setRight(config.stoneAboveGroundPosition)
    fun moveBothToStoneAboveGround() { moveLeftToStoneAboveGround(); moveRightToStoneAboveGround(); }
    fun moveStoneAboveGround() = moveBothToStoneAboveGround()

    fun update() {}
}