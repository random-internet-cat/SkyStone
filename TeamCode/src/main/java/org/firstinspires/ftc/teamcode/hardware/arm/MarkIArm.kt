package org.firstinspires.ftc.teamcode.hardware.arm

import com.acmerobotics.dashboard.config.Config
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.Servo
import org.firstinspires.ftc.teamcode.util.*
import org.firstinspires.ftc.teamcode.util.units.*

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

    @Config("MarkIArm VerticalControl")
    data class VerticalControl(val motor: DcMotor) {
        companion object {
            private const val MANUAL_MOTOR_POWER = 0.8
            private const val AUTOMATIC_MOTOR_POWER = 0.6

            @JvmField
            public var _COLLECT_TICKS: Int = 0

            @JvmField
            public var _STAGE0_TICKS: Int = 400

            @JvmField
            public var _PER_STAGE_TICKS: Int = 730

            private val COLLECT_POSITION get() = EncoderPosition(_COLLECT_TICKS)
            private val STAGE0_POSITION get() = EncoderPosition(_STAGE0_TICKS)
            private val PER_STAGE_TICKS get() = EncoderTicks(_PER_STAGE_TICKS)
        }

        sealed class State {
            companion object {
                private const val MIN_BLOCK_HEIGHT = 0
                private const val MAX_BLOCK_HEIGHT = 6
            }

            abstract val nextDown: State?
            abstract val nextUp: State?
            abstract val position: EncoderPosition

            object CollectState : State() {
                override val nextDown: State? get() = null
                override val nextUp: State? get() = PlaceBlockState(MIN_BLOCK_HEIGHT)
                override val position: EncoderPosition
                    get() = COLLECT_POSITION
            }

            data class PlaceBlockState(val blockHeight: Int) : State() {
                init {
                    require(blockHeight >= MIN_BLOCK_HEIGHT && blockHeight <= MAX_BLOCK_HEIGHT)
                }

                override val nextDown: State?
                    get() = if (blockHeight == MIN_BLOCK_HEIGHT) CollectState else PlaceBlockState(blockHeight - 1)

                override val nextUp: State?
                    get() = if (blockHeight == MAX_BLOCK_HEIGHT) null else PlaceBlockState(blockHeight + 1)

                override val position: EncoderPosition
                    get() = STAGE0_POSITION + blockHeight * PER_STAGE_TICKS
            }
        }

        private var _state: State

        /**
         * @return The most recent (or current) automatic state, regardless of if currently in automatic
         */
        public fun mostRecentAutomaticState() = _state

        /**
         * @return If in automatic, the current state, otherwise null
         */
        public fun currentAutomaticState() = if (!isManual) _state else null

        init {
            motor.resetEncoder()
            motor.targetPosition = 0
            _state = State.CollectState
        }

        private var _mutableIsManual: Boolean = true

        private val isManual get() = _mutableIsManual
        private val isAutomatic get() = !isManual

        private fun manuallyMoveWithPower(power: Double) {
            markManual()
            motor.power = power
        }

        private fun markManual() {
            _mutableIsManual = true
        }

        private fun markAutomatic() {
            _mutableIsManual = false
        }

        fun manuallyMoveUp() {
            manuallyMoveWithPower(MANUAL_MOTOR_POWER)
        }

        fun manuallyMoveDown() {
            manuallyMoveWithPower(-MANUAL_MOTOR_POWER)
        }

        fun stop() {
            motor.power = 0.0
        }

        fun switchToManual() {
            manuallyMoveWithPower(0.0)
        }

        fun stopIfManual() {
            if (isManual) stop()
        }

        private fun setMotorRunMode(runMode: DcMotor.RunMode) {
            motor.mode = runMode
        }

        private fun moveToPosition(encoderPosition: EncoderPosition) {
            motor.targetPosition = encoderPosition.raw
            motor.power = AUTOMATIC_MOTOR_POWER
            setMotorRunMode(DcMotor.RunMode.RUN_TO_POSITION)
        }

        private fun setCurrentState(state: State) {
            markAutomatic()
            _state = state
        }

        private fun moveToCurrentState() {
            val currentState = currentAutomaticState()
            require(currentState != null) { "Request to move to state in manual mode" }
            moveToPosition(currentState.position)
        }

        private fun moveToCurrentStateIfAutomatic() {
            if (isAutomatic) moveToCurrentState()
        }

        fun moveToState(state: State) {
            setCurrentState(state)
            moveToCurrentState()
        }

        fun isMovingToState(): Boolean {
            check(isAutomatic)
            check(motor.mode == DcMotor.RunMode.RUN_TO_POSITION)
            return motor.isBusy
        }

        fun update() {
            moveToCurrentStateIfAutomatic()
        }
    }

    @Config
    class Clamp(val servo: Servo) {
        companion object {
            @JvmField
            public var OPEN_POSITION = 0.8

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