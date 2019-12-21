package org.firstinspires.ftc.teamcode.hardware.arm

import org.firstinspires.ftc.teamcode.util.units.*

abstract class SeqStepState

private typealias SeqStepTick = (MarkIArm, SeqStepState) -> Boolean // Returns true if should keep moving

data class TickImpl<State : SeqStepState>(val initialState: () -> State, private val tickImpl: (MarkIArm, State) -> Boolean) {
    val tick: SeqStepTick = { arm, state -> tickImpl(arm, state as State) }
}

private object SeqStepEmptyState : SeqStepState()

private val noState = { SeqStepEmptyState }

fun moveArmStep(state: MarkIArm.VerticalControl.State): TickImpl<*> {
    return TickImpl(
        noState,
        { arm, _ -> arm.vertical.moveToState(state); arm.vertical.isMovingToState() }
    )
}

private data class StartTimeState(val startTime: SystemTime) : SeqStepState() {
    companion object {
        val current = { StartTimeState(currentSystemTime()) }
    }
}

private val CLOSE_CLAMP_RUNTIME = Seconds(1)
private val OPEN_CLAMP_RUNTIME = Seconds(1)

fun closeClampStep(): TickImpl<*> {
    return TickImpl(
        StartTimeState.current,
        { arm, state -> arm.clamp.close(); currentSystemTime() > state.startTime + CLOSE_CLAMP_RUNTIME }
    )
}

fun openClampStep(): TickImpl<*> {
    return TickImpl(
        StartTimeState.current,
        { arm, state -> arm.clamp.open(); currentSystemTime() > state.startTime + OPEN_CLAMP_RUNTIME }
    )
}

interface SeqStep {
    val tickImpl: TickImpl<*>
    val hasNext: Boolean
    val next: SeqStep
}

private fun SeqStep.initialState() = this.tickImpl.initialState()
private fun SeqStep.tick(arm: MarkIArm, state: SeqStepState) = this.tickImpl.tick(arm, state)

data class SeqStepList(val steps: List<TickImpl<*>>) {
    constructor(vararg stepArr: TickImpl<*>) : this(stepArr.toList())

    init {
        require(steps.isNotEmpty())
    }

    private inner class StepImpl(val index: Int) : SeqStep {
        init {
            require(index < steps.size)
        }

        override val tickImpl: TickImpl<*> = steps[index]

        override val hasNext: Boolean get() = index != steps.size
        override val next: SeqStep get() = StepImpl(index + 1)
    }

    fun first(): SeqStep = StepImpl(0)
}

data class AutomatedArmImpl(private val raw: MarkIArm) {
    private sealed class NextState {
        object Done : NextState()
        object Continue : NextState()
        data class SetNext(val next: ArmState) : NextState()
    }

    private sealed class ArmState {
        abstract val isManual: Boolean
        abstract fun update(arm: MarkIArm): NextState /* next */

        object ManualState : ArmState() {
            override val isManual: Boolean = true

            override fun update(arm: MarkIArm): NextState {
                return NextState.Continue
            }
        }

        data class SeqState(val step: SeqStep) : ArmState() {
            override val isManual: Boolean = false

            private val state = step.initialState()

            override fun update(arm: MarkIArm): NextState {
                val shouldContinue = step.tick(arm, state)

                if (shouldContinue) return NextState.Continue /* stay here */
                if (!step.hasNext) return NextState.Done
                return NextState.SetNext(SeqState(step.next))
            }
        }
    }

    private var state: ArmState = ArmState.ManualState

    fun update() {
        val nextState = state.update(raw)

        state = when(nextState) {
            is NextState.Done -> ArmState.ManualState
            is NextState.Continue -> state
            is NextState.SetNext -> nextState.next
        }

        raw.update()
    }

    fun isManual(): Boolean {
        return state.isManual
    }

    fun manualControl(): MarkIArm {
        check(isManual())
        return raw
    }

    fun isAutomatic(): Boolean {
        return !isManual()
    }

    fun startSeq(step: SeqStep) {
        state = ArmState.SeqState(step)
    }

    fun startSeq(list: SeqStepList) = startSeq(list.first())
}
