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
        public var CLAW_STONE_POSITION: Double = 0.01

        @JvmField
        public var CLAW_RETRACT_POSITION: Double = 0.18

        @JvmField
        public var CLAW_HIDDEN_POSITION: Double = 0.3

        // Clamp

        @JvmField
        public var CLAMP_GRAB_POSITION: Double = 0.99

        @JvmField
        public var CLAMP_UNCLAMP_POSITION: Double = 0.87
    }

    // Clamp

    private fun moveClampLeft(newPosition: Double) { leftClamp.position = newPosition }

    private fun moveClampRight(newPosition: Double) { rightClamp.position = newPosition - 0.325 } // why is this offset necessary? who knows!

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

    private fun moveClawRight(newPosition: Double) { rightClaw.position = newPosition + .325 } // why is this offset necessary? who knows!

    fun alignLeft() { moveClawLeft(CLAW_STONE_POSITION) }

    fun retractLeft() { moveClawLeft(CLAW_RETRACT_POSITION) }

    fun hideLeft() { moveClawLeft(CLAW_HIDDEN_POSITION) }

    fun alignRight() { moveClawRight(CLAW_STONE_POSITION) }

    fun retractRight() { moveClawRight(CLAW_RETRACT_POSITION) }

    fun hideRight() { moveClawRight(CLAW_HIDDEN_POSITION)}

    fun alignBoth() {
        alignLeft()
        alignRight()
    }

    fun retractBoth() {
        retractLeft()
        retractRight()
    }

    fun hideBoth() {
        // claw
        hideLeft()
        hideRight()

        // clamp
        clampLeft()
        clampRight()
    }

    fun update() {}
}