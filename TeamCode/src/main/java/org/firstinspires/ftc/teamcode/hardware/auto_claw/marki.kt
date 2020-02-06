package org.firstinspires.ftc.teamcode.hardware.auto_claw

import com.acmerobotics.dashboard.config.Config
import com.qualcomm.robotcore.hardware.Servo

@Config
data class MarkIAutoClaws(val leftClaw: Servo, val rightClaw: Servo, val leftClamp: Servo, val rightClamp: Servo) {

    /*    Functions belonging to each actuator:
        * Clamp: Clamp & Release
        * Claw: Align & Retract
    */

    companion object {
        // Claw

        @JvmField
        public var CLAW_STONE_POSITION: Double = 0.025

        @JvmField
        public var CLAW_RELEASE_POSITION: Double = 0.18

        // Clamp

        @JvmField
        public var CLAMP_GRAB_POSITION: Double = 0.99

        @JvmField
        public var CLAMP_UNCLAMP_POSITION: Double = 0.87
    }

    // Clamp

    private fun moveClampLeft(newPosition: Double) { leftClamp.position = newPosition }

    private fun moveClampRight(newPosition: Double) { rightClamp.position = newPosition }

    fun clampLeft() { moveClampLeft(CLAMP_GRAB_POSITION) }

    fun releaseLeft() { moveClampLeft(CLAMP_UNCLAMP_POSITION) }

    fun clampRight() { moveClampRight(CLAMP_GRAB_POSITION) }

    fun releaseRight() { moveClampRight(CLAMP_UNCLAMP_POSITION) }

    fun clampBoth() {
        clampLeft()
        clampRight()
    }

    fun releaseBoth() {
        releaseLeft()
        releaseRight()
    }

    // Claw

    private fun moveClawLeft(newPosition: Double) { leftClaw.position = newPosition }

    private fun moveClawRight(newPosition: Double) { rightClaw.position = newPosition }

    fun alignLeft() { moveClawLeft(CLAW_STONE_POSITION) }

    fun retractLeft() { moveClawLeft(CLAW_RELEASE_POSITION) }

    fun alignRight() { moveClawRight(CLAW_STONE_POSITION) }

    fun retractRight() { moveClawRight(CLAW_RELEASE_POSITION) }

    fun alignBoth() {
        alignLeft()
        alignRight()
    }

    fun retractBoth() {
        retractLeft()
        retractRight()
    }

    fun update() {}
}