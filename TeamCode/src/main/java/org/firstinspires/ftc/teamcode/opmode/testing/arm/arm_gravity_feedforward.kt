package org.firstinspires.ftc.teamcode.opmode.testing.arm

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.dashboard.config.ValueProvider
import com.acmerobotics.dashboard.config.variable.BasicVariable
import com.acmerobotics.dashboard.config.variable.CustomVariable
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.hardware.DcMotor
import org.firstinspires.ftc.teamcode.hardware.arm.PrototypeArm
import org.firstinspires.ftc.teamcode.hardware.arm.RRArmRotator
import org.firstinspires.ftc.teamcode.util.*
import org.firstinspires.ftc.teamcode.util.units.toDegrees

@Autonomous
@Config
class ArmGravityTuner : LinearOpMode() {
    companion object {
        private const val PID_VAR_NAME = "PID"
    }

    val catName: String = javaClass.simpleName

    private fun addDashboardVar(dashboard: FtcDashboard, arm: RRArmRotator): CustomVariable {
        val catVar = dashboard.configRoot.getVariable(catName) as CustomVariable? ?: run {
            val newVar = CustomVariable()
            dashboard.configRoot.putVariable(catName, newVar)
            newVar
        }

        val pidVar = CustomVariable()

        pidVar.putVariable("angleFF", BasicVariable(object : ValueProvider<Double> {
            override fun get(): Double {
                return arm.angleFeedforward()
            }

            override fun set(value: Double) {
                arm.setAngleFeedforward(value)
            }
        }))

        catVar.putVariable(PID_VAR_NAME, pidVar)
        dashboard.updateConfig()

        return catVar
    }

    private fun removeDashboardVar(dashboard: FtcDashboard, catVar: CustomVariable) {
        if (catVar.size() > 1) {
            catVar.removeVariable(PID_VAR_NAME)
        } else {
            dashboard.configRoot.removeVariable(catName)
        }

        dashboard.updateConfig()
    }

    override fun runOpMode() {
        val dashboard = FtcDashboard.getInstance()
        val motor = TypedMotor(hardwareMap.getMotor("arm"), externalGearing = 1.0)
        val arm = PrototypeArm.Rotator(motor).roadrunner()
        val dashboardVar = addDashboardVar(dashboard, arm)

        telemetry = MultipleTelemetry(telemetry, dashboard.getTelemetry())

        waitForStart()

        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT)

        while (opModeIsActive()) {
            if (gamepad1.a) {
                motor.setPower(0)
            } else {
                arm.update()
            }

            telemetry.addData("Angle", arm.currentAngle().toDegrees().raw)
            telemetry.update()
        }

        removeDashboardVar(dashboard, dashboardVar)
    }
}