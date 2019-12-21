package org.firstinspires.ftc.teamcode.hardware.arm;

import static org.firstinspires.ftc.teamcode.hardware.arm.Marki_automated_implKt.*;

public class MarkIAutomatedArm {
    private final AutomatedArmImpl impl;

    public static final SeqStepList OUTTAKE_STEPS = new SeqStepList(
        closeClampStep(),
        moveArmStep(new MarkIArm.VerticalControl.State.PlaceBlockState(2)),
        moveArmStep(new MarkIArm.VerticalControl.State.PlaceBlockState(0))
    );

    public static final SeqStepList INTAKE_STEPS = new SeqStepList(
        closeClampStep(),
        moveArmStep(MarkIArm.VerticalControl.State.CollectState.INSTANCE),
        openClampStep()
    );

    public MarkIAutomatedArm(MarkIArm rawArm) {
        this.impl = new AutomatedArmImpl(rawArm);
    }

    public boolean isAutomatic() {
        return impl.isAutomatic();
    }

    public boolean isManual() {
        return impl.isManual();
    }

    public MarkIArm manualControl() {
        return impl.manualControl();
    }

    public void outtakeSequence() {
        impl.startSeq(OUTTAKE_STEPS);
    }

    public void intakeSequence() {
        impl.startSeq(INTAKE_STEPS);
    }
}
