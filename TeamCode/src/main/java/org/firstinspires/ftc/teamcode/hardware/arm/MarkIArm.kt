package org.firstinspires.ftc.teamcode.hardware.arm

import com.acmerobotics.dashboard.config.Config
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.Servo
import org.firstinspires.ftc.teamcode.util.*
import org.firstinspires.ftc.teamcode.util.roadrunner.PIDCoefficients
import org.firstinspires.ftc.teamcode.util.roadrunner.roadrunner
import org.firstinspires.ftc.teamcode.util.units.*
import kotlin.math.PI

private typealias MarkIRotatorPosition = Int
private typealias MarkIWristPosition = Double

class MarkIArm(val rotator: Rotator, val wrist: Wrist, val clamp: Clamp) {
    @Config
    class Rotator(val motor: TypedMotorEx) {
        companion object {
            @JvmField
            public var _COLLECT_ANGLE_DEG: Int = 0

            private val COLLECT_ANGLE get() = DegreesPoint(_COLLECT_ANGLE_DEG)

            @JvmField
            public var _STAGE0_ANGLE_DEG: Int = 160

            private val STAGE0_ANGLE get() = DegreesPoint(_STAGE0_ANGLE_DEG)

            @JvmField
            public var _STAGE1_ANGLE_DEG: Int = 110

            private val STAGE1_ANGLE get() = DegreesPoint(_STAGE1_ANGLE_DEG)
        }

        init {
            motor.resetEncoder() // Assume start angle is 0
            motor.setTargetPosition(motor.encoderPosition())
            motor.motor.mode = DcMotor.RunMode.RUN_TO_POSITION
        }

        private fun moveToAngle(angle: DegreesPoint) {
            motor.setTargetPosition(motor.config.angleToEncoder(angle))
        }

        fun currentAngle() = motor.anglePosition()
        fun targetAngle() = motor.targetAnglePosition()

        fun moveToCollect() {
            moveToAngle(COLLECT_ANGLE)
            motor.setPower(0.6)
        }

        fun moveToStage0() {
            moveToAngle(STAGE0_ANGLE)
        }

        fun moveToStage1() {
            moveToAngle(STAGE1_ANGLE)
        }

        inline fun update() {}
    }

    @Config
    class Wrist(val servo: Servo) {
        companion object {
            @JvmField
            public var COLLECT_POSITION: MarkIWristPosition = 0.7

            @JvmField
            public var STAGE0_POSITION: MarkIWristPosition = 0.0

            @JvmField
            public var STAGE1_POSITION: MarkIWristPosition = 0.0
        }

        private fun setPosition(position: MarkIWristPosition) {
            servo.setPosition(position)
        }

        fun moveToCollect() {
            setPosition(COLLECT_POSITION)
        }

        fun moveToStage0() {
            setPosition(STAGE0_POSITION)
        }

        fun moveToStage1() {
            setPosition(STAGE1_POSITION)
        }

        inline fun update() {}
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

        inline fun update() {}
    }

    fun update() {
        rotator.update()
        wrist.update()
        clamp.update()
    }
}
