package org.firstinspires.ftc.teamcode.OpModes.ComponentTest.shooter;

import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Components.Shooter;

/**
 * STEP 1 for the flywheel. Finds:
 *   RIGHT_FLYWHEEL_REVERSED  (by eye, in manual mode)
 *   FLYWHEEL_KF              (from the sweep)
 *   FLYWHEEL_MAX_RPM         (from the sweep)
 *
 * Plain motor power, no PID, so these numbers only depend on the motors.
 *
 * Manual mode (gamepad 1):
 *   A          = flywheels on / off
 *   Dpad up    = power + 0.05
 *   Dpad down  = power - 0.05
 *   X          = run the sweep
 *
 * The sweep runs the flywheels at 0.3, 0.4 ... 1.0 power, waits for each to settle,
 * then averages the RPM. Keep hands and balls clear. B after the sweep goes back to manual.
 */
@Configurable
@TeleOp(name = "Shooter 1: Flywheel kF", group = "Shooter")
public class FlywheelFeedforwardTest extends LinearOpMode {

    private static final double[] SWEEP_POWERS = {0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0};
    public static double SETTLE_SEC = 2.5;   // time to reach steady speed at each power
    public static double SAMPLE_SEC = 1.0;   // time spent averaging after that
    public static double MAX_RPM_HEADROOM = 0.9;  // leave room for the PID to push harder after a shot

    private DcMotorEx left, right;

    @Override
    public void runOpMode() {
        // Every telemetry line goes to both the Driver Station and Panels.
        telemetry = new JoinedTelemetry(PanelsTelemetry.INSTANCE.getFtcTelemetry(), telemetry);

        left = hardwareMap.get(DcMotorEx.class, Shooter.LEFT_FLYWHEEL_NAME);
        right = hardwareMap.get(DcMotorEx.class, Shooter.RIGHT_FLYWHEEL_NAME);
        // Same setup as the Shooter class, so the readings match what it will see.
        left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        right.setDirection(Shooter.RIGHT_FLYWHEEL_REVERSED
                ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);

        telemetry.addLine("Shooter 1: Flywheel kF. Keep hands clear of the flywheels.");
        telemetry.update();
        waitForStart();

        double power = 0.3;
        boolean on = false;
        double[][] results = null;   // [step][0 = left rpm, 1 = right rpm]

        while (opModeIsActive()) {
            if (results != null) {
                showResults(results);
                if (gamepad1.bWasPressed()) results = null;
                telemetry.update();
                continue;
            }

            if (gamepad1.aWasPressed()) on = !on;
            if (gamepad1.dpadUpWasPressed()) power = Range.clip(power + 0.05, 0, 1);
            if (gamepad1.dpadDownWasPressed()) power = Range.clip(power - 0.05, 0, 1);
            if (gamepad1.xWasPressed()) {
                results = runSweep();
                on = false;
                continue;
            }

            setPower(on ? power : 0);

            telemetry.addLine("MANUAL  (A on/off, dpad power, X sweep)");
            telemetry.addData("Power", "%.2f  %s", power, on ? "ON" : "off");
            addEncoderLines(on ? power : 0);
            telemetry.addLine();
            telemetry.addLine("Check by eye: both wheels should throw a ball OUT of the robot.");
            telemetry.addLine("Right wheel wrong way -> flip RIGHT_FLYWHEEL_REVERSED.");
            telemetry.addLine("Left wheel wrong way -> Shooter has no setting for that yet,");
            telemetry.addLine("  swap the motor's wires or add a LEFT_FLYWHEEL_REVERSED.");
            telemetry.addLine("RPM reading negative while spinning -> encoder is backwards.");
            telemetry.update();
        }
        setPower(0);
    }

    /** Runs every sweep power and returns the steady RPM of each wheel. Returns early if stopped. */
    private double[][] runSweep() {
        double[][] out = new double[SWEEP_POWERS.length][2];
        ElapsedTime timer = new ElapsedTime();

        for (int i = 0; i < SWEEP_POWERS.length && opModeIsActive(); i++) {
            double p = SWEEP_POWERS[i];
            setPower(p);
            timer.reset();
            double sumL = 0, sumR = 0;
            int n = 0;
            while (opModeIsActive() && timer.seconds() < SETTLE_SEC + SAMPLE_SEC) {
                double l = leftRpm(), r = rightRpm();
                if (timer.seconds() > SETTLE_SEC) {
                    sumL += l;
                    sumR += r;
                    n++;
                }
                telemetry.addLine("SWEEP running, keep clear");
                telemetry.addData("Step", "%d of %d   power %.2f", i + 1, SWEEP_POWERS.length, p);
                telemetry.addData("Phase", timer.seconds() < SETTLE_SEC ? "settling" : "measuring");
                addEncoderLines(p);
                telemetry.update();
            }
            out[i][0] = n > 0 ? sumL / n : 0;
            out[i][1] = n > 0 ? sumR / n : 0;
        }
        setPower(0);
        return out;
    }

    private void showResults(double[][] r) {
        // Shooter uses power = KF * rpm with no offset, so fit a line through zero:
        //   KF = sum(power * rpm) / sum(rpm^2)
        double pl = 0, ll = 0, pr = 0, rr = 0, pa = 0, aa = 0;
        for (int i = 0; i < r.length; i++) {
            double p = SWEEP_POWERS[i];
            double avg = (r[i][0] + r[i][1]) / 2.0;
            pl += p * r[i][0]; ll += r[i][0] * r[i][0];
            pr += p * r[i][1]; rr += r[i][1] * r[i][1];
            pa += p * avg;     aa += avg * avg;
        }
        double kfLeft = ll > 0 ? pl / ll : 0;
        double kfRight = rr > 0 ? pr / rr : 0;
        double kf = aa > 0 ? pa / aa : 0;

        double[] last = r[r.length - 1];
        double fullPowerRpm = Math.min(last[0], last[1]);

        telemetry.addLine("SWEEP DONE  (B = back to manual)");
        for (int i = 0; i < r.length; i++) {
            telemetry.addData(String.format("power %.1f", SWEEP_POWERS[i]),
                    "L %.0f   R %.0f", r[i][0], r[i][1]);
        }
        telemetry.addData("kF left / right", "%.3e / %.3e", kfLeft, kfRight);
        if (kfLeft > 0 && kfRight > 0 && Math.abs(kfLeft - kfRight) / kf > 0.1) {
            telemetry.addLine("Wheels differ by >10%. Check for rubbing or a loose belt.");
        }
        telemetry.addData("Full power RPM (slower wheel)", "%.0f", fullPowerRpm);
        telemetry.addLine();
        telemetry.addLine("Paste into Shooter.java:");
        telemetry.addLine(String.format("FLYWHEEL_KF = %.6e;", kf));
        telemetry.addLine(String.format("FLYWHEEL_MAX_RPM = %.0f;", fullPowerRpm * MAX_RPM_HEADROOM));
        telemetry.addLine();
        telemetry.addLine("If full-power RPM is far from the motor's rated speed,");
        telemetry.addLine("FLYWHEEL_TICKS_PER_REV is probably wrong (28 = bare motor).");
    }

    /**
     * RPM plus the raw encoder count and port of each motor. If the count stays at 0 while
     * a wheel is clearly spinning, that motor's encoder isn't being read: check the encoder
     * cable is plugged into the SAME port number as the motor's power cable.
     * RPM values are plain numbers so Panels can graph them.
     */
    private void addEncoderLines(double commandedPower) {
        double l = leftRpm(), r = rightRpm();
        telemetry.addData("leftRpm", l);
        telemetry.addData("rightRpm", r);
        telemetry.addData("power", commandedPower);
        telemetry.addData("Left encoder count / port", "%d / %d",
                left.getCurrentPosition(), left.getPortNumber());
        telemetry.addData("Right encoder count / port", "%d / %d",
                right.getCurrentPosition(), right.getPortNumber());
        if (commandedPower > 0.2) {
            if (Math.abs(l) < 50) telemetry.addLine("LEFT encoder not reading: check its encoder cable/port.");
            if (Math.abs(r) < 50) telemetry.addLine("RIGHT encoder not reading: check its encoder cable/port.");
        }
    }

    private void setPower(double p) {
        left.setPower(p);
        right.setPower(p);
    }

    private double leftRpm() {
        return left.getVelocity() / Shooter.FLYWHEEL_TICKS_PER_REV * 60.0;
    }

    private double rightRpm() {
        return right.getVelocity() / Shooter.FLYWHEEL_TICKS_PER_REV * 60.0;
    }
}
