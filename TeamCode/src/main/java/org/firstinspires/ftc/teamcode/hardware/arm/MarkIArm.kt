package org.firstinspires.ftc.teamcode.hardware.arm

import com.acmerobotics.dashboard.config.Config
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.Servo
import org.firstinspires.ftc.teamcode.util.*
import org.firstinspires.ftc.teamcode.util.roadrunner.PIDCoefficients
import org.firstinspires.ftc.teamcode.util.roadrunner.roadrunner
import org.firstinspires.ftc.teamcode.util.units.*
import kotlin.math.PI

data class MarkIArm(val horizontal: HorizontalControl, val vertical: VerticalControl, val clamp: Clamp) {
    data class HorizontalControl(val motor: DcMotor) {
        companion object {
            private const val MOTOR_POWER = 0.8
        }

        private fun power(power: Double) {
            motor.power = power
        }

        private fun power(power: Int) = power(power.toDouble())

        fun moveOut() {
            power(MOTOR_POWER)
        }

        fun moveIn() {
            power(-MOTOR_POWER)
        }

        fun stop() {
            power(0)
        }

        fun update() {}
    }

    data class VerticalControl(val motor: DcMotor) {
        companion object {
            private const val MOTOR_POWER = 0.8
        }

        private fun power(power: Double) {
            motor.power = power
        }

        private fun power(power: Int) = power(power.toDouble())

        fun moveUp() {
            power(MOTOR_POWER)
        }

        fun moveDown() {
            power(-MOTOR_POWER)
        }

        fun stop() {
            power(0)
        }

        fun update() {}
    }

    @Config
    class Clamp(val servo: Servo) {
        companion object {
            @JvmField
            public var OPEN_POSITION = 0.0

            @JvmField
            public var CLOSED_POSITION = 0.2
        }

        fun open() {
            servo.position = OPEN_POSITION
        }

        fun close() {
            servo.position = CLOSED_POSITION
        }

        fun servoPosition() = servo.position

        fun update() {}
    }

    fun update() {
        horizontal.update()
        vertical.update()
        clamp.update()
    }
}