package org.firstinspires.ftc.teamcode.OpModes.ComponentTest.shooter;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Components.Shooter;
import org.firstinspires.ftc.teamcode.Wrapper.PidController;

/**
 * STEP 2 for the turret. Finds TURRET_KP, TURRET_KI, TURRET_KD and TURRET_MAX_POWER.
 * Do "Shooter 4: Turret Setup" first and paste its numbers in, or the angles here are wrong.
 *
 * Point the turret at TURRET_START_DEG (forward, if you used the setup test's numbers)
 * BEFORE pressing INIT. The encoder count resets at init, same as the Shooter class.
 *
 * Same control as Shooter: power = TURRET_DIRECTION * PID(target - angle).
 *
 * Gamepad 1:
 *   A                 = swing between forward - step and forward + step
 *   B                 = go back to forward
 *   Dpad up / down    = step size +/- 5 deg
 *   Dpad left / right = pick a gain
 *   Bumpers           = picked gain x / 1.25
 *   Y                 = picked gain to 0
 *
 * How to tune: raise kP until each swing gets there quickly. If it overshoots and
 * comes back, add a little kD or lower kP. Add kI only if it stops short every time.
 * Watch "Settle time" and "Overshoot" and try both big and small steps.
 */
@TeleOp(name = "Shooter 5: Turret PID", group = "Shooter")
public class TurretPidTest extends LinearOpMode {

    private static final int KP = 0, KI = 1, KD = 2, MAX_POWER = 3;
    private static final double SETTLE_HOLD_SEC = 0.25;  // must stay in tolerance this long to count as settled

    @Override
    public void runOpMode() {
        DcMotorEx turret = hardwareMap.get(DcMotorEx.class, Shooter.TURRET_NAME);
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        TunableValues gains = new TunableValues(
                new String[]{"kP", "kI", "kD", "max power"},
                new double[]{Shooter.TURRET_KP, Shooter.TURRET_KI, Shooter.TURRET_KD, Shooter.TURRET_MAX_POWER},
                new double[]{0.01, 0.001, 0.0005, 0.3});
        PidController pid = new PidController(0, 0, 0);
        TelemetryManager panels = PanelsTelemetry.INSTANCE.getTelemetry();

        telemetry.addLine(String.format("Turret must be at %.0f deg (TURRET_START_DEG) right now.",
                Shooter.TURRET_START_DEG));
        telemetry.addLine("If it isn't, stop, move it there, and init again.");
        telemetry.update();
        waitForStart();

        double forward = Shooter.TURRET_FORWARD_DEG;
        double step = 30;
        boolean swingRight = true;
        double target = Range.clip(Shooter.TURRET_START_DEG, Shooter.TURRET_MIN_DEG, Shooter.TURRET_MAX_DEG);

        ElapsedTime loopTimer = new ElapsedTime();
        ElapsedTime moveTimer = new ElapsedTime();
        ElapsedTime inTolTimer = new ElapsedTime();
        boolean wasInTol = false;
        double moveStartDeg = target;
        double settleSec = -1, overshoot = 0;
        boolean settled = true;

        while (opModeIsActive()) {
            double dt = loopTimer.seconds();
            loopTimer.reset();

            gains.handle(gamepad1);
            double newTarget = target;
            if (gamepad1.aWasPressed()) {
                newTarget = swingRight ? forward + step : forward - step;
                swingRight = !swingRight;
            }
            if (gamepad1.bWasPressed()) newTarget = forward;
            if (gamepad1.dpadUpWasPressed()) step = Math.min(step + 5, 90);
            if (gamepad1.dpadDownWasPressed()) step = Math.max(step - 5, 5);
            newTarget = Range.clip(newTarget, Shooter.TURRET_MIN_DEG, Shooter.TURRET_MAX_DEG);

            double angle = Shooter.TURRET_START_DEG
                    + Shooter.TURRET_DIRECTION * turret.getCurrentPosition() / Shooter.TURRET_TICKS_PER_DEG;

            if (newTarget != target) {
                target = newTarget;
                moveStartDeg = angle;
                moveTimer.reset();
                settled = false;
                settleSec = -1;
                overshoot = 0;
                wasInTol = false;
            }

            // ---------- control (same as Shooter.update) ----------
            double maxPower = Math.min(gains.get(MAX_POWER), 1.0);
            pid.setGains(gains.get(KP), gains.get(KI), gains.get(KD));
            pid.setOutputLimits(-maxPower, maxPower);
            double out = dt > 0 ? pid.update(target, angle, dt) : 0;
            turret.setPower(Shooter.TURRET_DIRECTION * out);

            // ---------- measurements ----------
            double error = target - angle;
            boolean inTol = Math.abs(error) < Shooter.TURRET_TOLERANCE_DEG;
            if (!settled) {
                double moveDir = Math.signum(target - moveStartDeg);
                overshoot = Math.max(overshoot, moveDir * (angle - target));
                if (inTol && !wasInTol) inTolTimer.reset();
                if (inTol && inTolTimer.seconds() >= SETTLE_HOLD_SEC) {
                    settled = true;
                    settleSec = moveTimer.seconds() - SETTLE_HOLD_SEC;
                }
            }
            wasInTol = inTol;

            // ---------- telemetry ----------
            telemetry.addData("Target / angle (deg)", "%.1f / %.1f", target, angle);
            telemetry.addData("Error (deg)", "%.2f", error);
            telemetry.addData("Power", "%.2f", Shooter.TURRET_DIRECTION * out);
            telemetry.addData("Step size (deg)", "%.0f", step);
            telemetry.addLine();
            gains.addTelemetry(telemetry);
            telemetry.addLine();
            telemetry.addData("Settle time (s)", settled
                    ? (settleSec < 0 ? "-" : String.format("%.2f", settleSec)) : "moving...");
            telemetry.addData("Overshoot (deg)", "%.2f", overshoot);
            telemetry.addLine();
            telemetry.addLine("Paste into Shooter.java:");
            telemetry.addLine("TURRET_KP = " + TunableValues.fmt(gains.get(KP)) + ";");
            telemetry.addLine("TURRET_KI = " + TunableValues.fmt(gains.get(KI)) + ";");
            telemetry.addLine("TURRET_KD = " + TunableValues.fmt(gains.get(KD)) + ";");
            telemetry.addLine("TURRET_MAX_POWER = " + TunableValues.fmt(maxPower) + ";");
            telemetry.update();

            panels.addData("target", target);
            panels.addData("angle", angle);
            panels.update();
        }
        turret.setPower(0);
    }
}
