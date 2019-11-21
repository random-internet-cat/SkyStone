package org.firstinspires.ftc.teamcode.opmode.testing.arm

import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.roadrunner.tuning.AccelRegression
import com.acmerobotics.roadrunner.tuning.RampRegression
import com.acmerobotics.roadrunner.util.NanoClock
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.hardware.DcMotor

import org.firstinspires.ftc.robotcore.internal.system.Misc
import org.firstinspires.ftc.teamcode.hardware.arm.ArmMovementConstraints
import org.firstinspires.ftc.teamcode.hardware.arm.PrototypeArm
import org.firstinspires.ftc.teamcode.hardware.arm.power
import org.firstinspires.ftc.teamcode.util.*
import org.firstinspires.ftc.teamcode.util.units.*
import kotlin.math.sqrt

/*
 * Op mode for computing kV, kStatic, and kA from various arm motions. Note: for those using the
 * built-in PID, **kStatic and kA should not be tuned**. For the curious, here's an outline of the
 * basic procedure:
 *   1. Slowly ramp the motor power and record encoder values along the way.
 *   2. Run a linear regression on the encoder velocity vs. motor power plot to obtain a slope (kV)
 *      and an optional intercept (kStatic).
 *   3. Accelerate the arm (apply constant power) and record the encoder counts.
 *   4. Adjust the encoder data based on the velocity tuning data and find kA with another linear
 *      regression.
 */
@Config
@Autonomous(group = "arm")
class ArmFeedforwardTuner : LinearOpMode() {
    companion object {
        private const val MAX_POWER = 0.6
    }

    @Throws(InterruptedException::class)
    override fun runOpMode() {
        val armMotorRaw = hardwareMap.getMotor("arm")
        val externalGearing = 1.0
        val armMotor = TypedMotor(armMotorRaw, externalGearing)
        val maxRPM = RevolutionsPerMinute(armMotor.config.rawConfig.maxRPM)
        val movementConstraints = ArmMovementConstraints(RadiansPoint(TWO_PI), RevolutionsPerSecond(1.0), RevolutionsPerSecondSquared(1.0), RevolutionsPerSecondCubed(1.0))
        val maxAngle = movementConstraints.maxAngle

//        val arm = RRArmRotator(armMotor, PIDCoefficients(), DcMotorFeedforward(), 0.0, movementConstraints)
        val arm = PrototypeArm.Rotator(armMotor).roadrunner()
        armMotor.enableEncoder()
        armMotor.resetEncoder()
        armMotor.setTargetPosition(0)

        val clock = NanoClock.system()

        telemetry.log().add("Press play to begin the feedforward tuning routine")
        telemetry.update()

        waitForStart()

        if (isStopRequested) return

        telemetry.log().clear()
        telemetry.log().add("Would you like to fit kStatic?")
        telemetry.log().add("Press (A) for yes, (B) for no")
        telemetry.update()

        var fitIntercept = false
        while (!isStopRequested) {
            if (gamepad1.a) {
                fitIntercept = true
                while (!isStopRequested && gamepad1.a) {
                    idle()
                }
                break
            } else if (gamepad1.b) {
                while (!isStopRequested && gamepad1.b) {
                    idle()
                }
                break
            }
            idle()
        }

        telemetry.log().clear()
        telemetry.log().add("Press (A) to begin")
        telemetry.update()

        while (!isStopRequested && !gamepad1.a) {
            idle()
        }
        while (!isStopRequested && gamepad1.a) {
            idle()
        }

        telemetry.log().clear()
        telemetry.log().add("Running...")
        telemetry.update()

        val maxVel = RevolutionsPerSecond(maxRPM).raw * externalGearing * TWO_PI
        val finalVel = MAX_POWER * maxVel
        val accel = finalVel * finalVel / (2.0 * RadiansPoint(maxAngle).raw)
        val rampTime = sqrt(2.0 * RadiansPoint(maxAngle).raw / accel)

        var startTime = clock.seconds()
        val rampRegression = RampRegression()

        while (!isStopRequested) {
            val elapsedTime = clock.seconds() - startTime
            if (elapsedTime > rampTime) {
                break
            }
            val vel = accel * elapsedTime
            val power = vel / maxVel

            rampRegression.add(elapsedTime, arm.currentAngle().toRadians().raw, power)

            arm.power(power)
        }
        arm.power(0)

        val (kV, kStatic, rSquare) = rampRegression.fit(fitIntercept)

//        rampRegression.save(LoggingUtil.getLogFile(Misc.formatInvariant(
//            "ArmRampRegression-%d.csv", System.currentTimeMillis())))

        telemetry.log().clear()
        telemetry.log().add("Quasi-static ramp up test complete")
        if (fitIntercept) {
            telemetry.log().add(Misc.formatInvariant("kV = %.5f, kStatic = %.5f (R^2 = %.2f)",
                kV, kStatic, rSquare))
        } else {
            telemetry.log().add(Misc.formatInvariant("kV = %.5f (R^2 = %.2f)",
                kV, rSquare))
        }
        telemetry.log().add("Would you like to fit kA?")
        telemetry.log().add("Press (A) for yes, (B) for no")
        telemetry.update()

        var fitAccelFF = false
        while (!isStopRequested) {
            if (gamepad1.a) {
                fitAccelFF = true
                while (!isStopRequested && gamepad1.a) {
                    idle()
                }
                break
            } else if (gamepad1.b) {
                while (!isStopRequested && gamepad1.b) {
                    idle()
                }
                break
            }
            idle()
        }

        if (fitAccelFF) {
            telemetry.log().clear()
            telemetry.log().add("Press (A) to continue")
            telemetry.update()

            while (!isStopRequested && !gamepad1.a) {
                idle()
            }
            while (!isStopRequested && gamepad1.a) {
                idle()
            }

            telemetry.log().clear()
            telemetry.log().add("Running...")
            telemetry.update()

            val maxPowerTime = 0.75 * maxAngle.toRadians().raw / maxVel // 0.75 = "safety factor"

            startTime = clock.seconds()
            val accelRegression = AccelRegression()

            arm.power(-MAX_POWER)
            while (!isStopRequested) {
                val elapsedTime = clock.seconds() - startTime
                if (elapsedTime > maxPowerTime) {
                    break
                }

                accelRegression.add(elapsedTime, arm.currentAngle().toRadians().raw, -MAX_POWER)
            }
            arm.power(0)

            val (kA, rSquare1) = accelRegression.fit(
                kV, kStatic)

//            accelRegression.save(LoggingUtil.getLogFile(Misc.formatInvariant(
//                "ArmAccelRegression-%d.csv", System.currentTimeMillis())))

            telemetry.log().clear()
            telemetry.log().add("Constant power test complete")
            telemetry.log().add(Misc.formatInvariant("kA = %.5f (R^2 = %.2f)",
                kA, rSquare1))
            telemetry.update()
        }

        while (!isStopRequested) {
            idle()
        }
    }
}