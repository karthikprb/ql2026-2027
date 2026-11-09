package org.firstinspires.ftc.teamcode.OpModes.ComponentTest.shooter;

import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.configurables.annotations.Configurable;
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
 * Change the gains in PANELS while this runs, under "Shooter":
 *   TURRET_KP, TURRET_KI, TURRET_KD, TURRET_MAX_POWER, TURRET_TOLERANCE_DEG
 * They are read every loop. Panels doesn't save them: copy the final numbers into
 * Shooter.java. The swing size is STEP_DEG under "TurretPidTest".
 *
 * Point the turret at TURRET_START_DEG (forward, if you used the setup test's numbers)
 * BEFORE pressing INIT. The encoder count resets at init, same as the Shooter class.
 *
 * Same control as Shooter: power = TURRET_DIRECTION * PID(target - angle).
 *
 * Gamepad 1:
 *   A              = swing between forward - step and forward + step
 *   B              = go back to forward
 *   Dpad up / down = step size +/- 5 deg
 *
 * How to tune: raise kP until each swing gets there quickly. If it overshoots and
 * comes back, add a little kD or lower kP. Add kI only if it stops short every time.
 * Watch "Settle time" and "Overshoot" and try both big and small steps.
 */
@Configurable
@TeleOp(name = "Shooter 5: Turret PID", group = "Shooter")
public class TurretPidTest extends LinearOpMode {

    public static double STEP_DEG = 30;
    public static double SETTLE_HOLD_SEC = 0.25;  // must stay in tolerance this long to count as settled

    @Override
    public void runOpMode() {
        // Every telemetry line goes to both the Driver Station and Panels.
        telemetry = new JoinedTelemetry(PanelsTelemetry.INSTANCE.getFtcTelemetry(), telemetry);

        DcMotorEx turret = hardwareMap.get(DcMotorEx.class, Shooter.TURRET_NAME);
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        PidController pid = new PidController(0, 0, 0);

        telemetry.addLine(String.format("Turret must be at %.0f deg (TURRET_START_DEG) right now.",
                Shooter.TURRET_START_DEG));
        telemetry.addLine("If it isn't, stop, move it there, and init again.");
        telemetry.addLine("Change gains in Panels under \"Shooter\".");
        telemetry.update();
        waitForStart();

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

            double forward = Shooter.TURRET_FORWARD_DEG;
            double newTarget = target;
            if (gamepad1.aWasPressed()) {
                newTarget = swingRight ? forward + STEP_DEG : forward - STEP_DEG;
                swingRight = !swingRight;
            }
            if (gamepad1.bWasPressed()) newTarget = forward;
            if (gamepad1.dpadUpWasPressed()) STEP_DEG = Math.min(STEP_DEG + 5, 90);
            if (gamepad1.dpadDownWasPressed()) STEP_DEG = Math.max(STEP_DEG - 5, 5);
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
            double maxPower = Range.clip(Shooter.TURRET_MAX_POWER, 0.01, 1.0);
            pid.setGains(Shooter.TURRET_KP, Shooter.TURRET_KI, Shooter.TURRET_KD);
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
            telemetry.addData("Step size (deg)", "%.0f", STEP_DEG);
            telemetry.addLine();
            telemetry.addData("Settle time (s)", settled
                    ? (settleSec < 0 ? "-" : String.format("%.2f", settleSec)) : "moving...");
            telemetry.addData("Overshoot (deg)", "%.2f", overshoot);
            telemetry.addLine();
            telemetry.addLine("Gains now (edit in Panels > Shooter). Paste into Shooter.java:");
            telemetry.addLine("TURRET_KP = " + fmt(Shooter.TURRET_KP) + ";");
            telemetry.addLine("TURRET_KI = " + fmt(Shooter.TURRET_KI) + ";");
            telemetry.addLine("TURRET_KD = " + fmt(Shooter.TURRET_KD) + ";");
            telemetry.addLine("TURRET_MAX_POWER = " + fmt(Shooter.TURRET_MAX_POWER) + ";");
            telemetry.addLine();
            // Plain numbers for the Panels graph.
            telemetry.addData("target", target);
            telemetry.addData("angle", angle);
            telemetry.update();
        }
        turret.setPower(0);
    }

    private static String fmt(double v) {
        return String.format("%.6g", v);
    }
}
