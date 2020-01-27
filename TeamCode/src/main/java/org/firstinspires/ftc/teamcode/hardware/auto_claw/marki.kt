package org.firstinspires.ftc.teamcode.hardware.auto_claw

import com.acmerobotics.dashboard.config.Config
import com.qualcomm.robotcore.hardware.Servo

@Config
data class MarkIAutoClaws(val left: Servo, val right: Servo) {
    companion object {
        @JvmField
        public var CLAMP_STONE_POSITION: Double = 0.0

        @JvmField
        public var RELEASE_STONE_POSITION: Double = 0.7
    }

    fun moveLeft(newPosition: Double) {
        left.position = newPosition
    }

    fun moveRight(newPosition: Double) {
        right.position = newPosition
    }

    fun clampLeft() {
        moveLeft(CLAMP_STONE_POSITION)
    }

    fun releaseLeft() {
        moveLeft(RELEASE_STONE_POSITION)
    }

    fun clampRight() {
        moveRight(CLAMP_STONE_POSITION)
    }

    fun releaseRight() {
        moveRight(RELEASE_STONE_POSITION)
    }

    fun clampBoth() {
        clampLeft()
        clampRight()
    }

    fun releaseBoth() {
        releaseLeft()
        releaseRight()
    }

    fun update() {}
}