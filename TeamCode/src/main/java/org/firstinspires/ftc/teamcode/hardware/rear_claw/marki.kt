package org.firstinspires.ftc.teamcode.hardware.rear_claw

import com.acmerobotics.dashboard.config.Config
import com.qualcomm.robotcore.hardware.Servo

@Config
data class MarkIRearClaws(val left: Servo, val right: Servo) {
    companion object {
        @JvmField
        public val CLAMP_POSITION: Double = 0.0

        @JvmField
        public val RELEASE_POSITION: Double = 0.0
    }

    fun moveLeft(newPosition: Double) {
        left.position = newPosition
    }

    fun moveRight(newPosition: Double) {
        right.position = newPosition
    }

    fun clampLeft() {
        moveLeft(CLAMP_POSITION)
    }

    fun releaseLeft() {
        moveLeft(RELEASE_POSITION)
    }

    fun clampRight() {
        moveRight(CLAMP_POSITION)
    }

    fun releaseRight() {
        moveRight(RELEASE_POSITION)
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