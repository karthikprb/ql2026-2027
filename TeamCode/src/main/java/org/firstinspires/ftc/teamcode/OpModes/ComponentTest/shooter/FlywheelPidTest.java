package org.firstinspires.ftc.teamcode.OpModes.ComponentTest.shooter;

import com.bylazar.gamepad.PanelsGamepad;
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
import org.firstinspires.ftc.teamcode.Wrapper.PidController;

/**
 * STEP 2 for the flywheel. Finds FLYWHEEL_KP, FLYWHEEL_KI, FLYWHEEL_KD (and lets you
 * nudge FLYWHEEL_KF). Do "Shooter 1: Flywheel kF" first and paste its kF in.
 *
 * Change the gains in PANELS while this runs, under "Shooter":
 *   FLYWHEEL_KP, FLYWHEEL_KI, FLYWHEEL_KD, FLYWHEEL_KF
 * They are read every loop, so a change applies immediately. Panels doesn't save them:
 * copy the final numbers into Shooter.java. The target RPM is under "FlywheelPidTest".
 *
 * Same control as the Shooter class: power = kF * target + PID(target - rpm).
 *
 * Gamepad 1:
 *   A              = flywheels on / off (spin-up test)
 *   Dpad up / down = target RPM +/- 250
 *
 * How to tune:
 *   1. kP 0, press A. With only kF the RPM should settle a little under target.
 *   2. Raise kP until it gets to target fast. If it overshoots or wobbles, back off.
 *   3. Feed a ball through. "Last dip" and "Recovery" show how hard the shot hit and
 *      how long it took to get back. More kP = faster recovery, until it wobbles.
 *   4. Only add kI if it always sits a bit under target. Only add kD if it wobbles.
 * Graph target / left / right in Panels to see it.
 */
@Configurable
@TeleOp(name = "Shooter 2: Flywheel PID", group = "Shooter")
public class FlywheelPidTest extends LinearOpMode {

    public static double TARGET_RPM = 3500;

    private DcMotorEx left, right;

    @Override
    public void runOpMode() {
        // Every telemetry line goes to both the Driver Station and Panels.
        telemetry = new JoinedTelemetry(PanelsTelemetry.INSTANCE.getFtcTelemetry(), telemetry);
        // Gamepad works from the Driver Station or a controller plugged into the Panels computer.
        PanelsPad pad1 = new PanelsPad(gamepad1, PanelsGamepad.INSTANCE.getFirstManager());

        left = hardwareMap.get(DcMotorEx.class, Shooter.LEFT_FLYWHEEL_NAME);
        right = hardwareMap.get(DcMotorEx.class, Shooter.RIGHT_FLYWHEEL_NAME);
        left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        right.setDirection(Shooter.RIGHT_FLYWHEEL_REVERSED
                ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);

        PidController leftPid = new PidController(0, 0, 0);
        PidController rightPid = new PidController(0, 0, 0);
        leftPid.setOutputLimits(-1, 1);
        rightPid.setOutputLimits(-1, 1);


        telemetry.addLine("Shooter 2: Flywheel PID. Keep hands clear.");
        telemetry.addLine("Change gains in Panels under \"Shooter\".");
        telemetry.update();
        waitForStart();

        boolean on = false;

        ElapsedTime loopTimer = new ElapsedTime();
        ElapsedTime stepTimer = new ElapsedTime();

        // Spin-up results
        double riseSec = -1;        // time from A to first time at speed
        double overshoot = 0;       // highest RPM above target after reaching it
        boolean reached = false;
        double lastTarget = TARGET_RPM;

        // Shot (dip) results
        boolean inDip = false;
        double dipLowest = 0;
        double lastDip = 0;         // how far below target the last dip went
        double lastRecoverySec = -1;
        ElapsedTime dipTimer = new ElapsedTime();

        while (opModeIsActive()) {
            pad1.update();
            double dt = loopTimer.seconds();
            loopTimer.reset();

            // ---------- buttons ----------
            boolean restart = false;
            if (pad1.aWasPressed()) {
                on = !on;
                restart = true;
            }
            if (pad1.dpadUpWasPressed()) TARGET_RPM += 250;
            if (pad1.dpadDownWasPressed()) TARGET_RPM -= 250;
            TARGET_RPM = Range.clip(TARGET_RPM, 0, Shooter.FLYWHEEL_MAX_RPM);
            double target = TARGET_RPM;
            if (target != lastTarget) restart = true;   // new target from dpad or Panels
            lastTarget = target;

            if (restart) {
                stepTimer.reset();
                riseSec = -1;
                overshoot = 0;
                reached = false;
                inDip = false;
                leftPid.reset();
                rightPid.reset();
            }

            // ---------- control (same as Shooter.update) ----------
            double l = left.getVelocity() / Shooter.FLYWHEEL_TICKS_PER_REV * 60.0;
            double r = right.getVelocity() / Shooter.FLYWHEEL_TICKS_PER_REV * 60.0;
            double avg = (l + r) / 2.0;

            leftPid.setGains(Shooter.FLYWHEEL_KP, Shooter.FLYWHEEL_KI, Shooter.FLYWHEEL_KD);
            rightPid.setGains(Shooter.FLYWHEEL_KP, Shooter.FLYWHEEL_KI, Shooter.FLYWHEEL_KD);

            double leftPower = 0, rightPower = 0;
            if (on && dt > 0) {
                double ff = Shooter.FLYWHEEL_KF * target;
                leftPower = Range.clip(ff + leftPid.update(target, l, dt), 0, 1);
                rightPower = Range.clip(ff + rightPid.update(target, r, dt), 0, 1);
            }
            left.setPower(leftPower);
            right.setPower(rightPower);

            // ---------- measurements ----------
            boolean atSpeed = on
                    && Math.abs(l - target) < Shooter.RPM_TOLERANCE
                    && Math.abs(r - target) < Shooter.RPM_TOLERANCE;
            if (on) {
                if (!reached && atSpeed) {
                    reached = true;
                    riseSec = stepTimer.seconds();
                }
                if (reached) {
                    overshoot = Math.max(overshoot, avg - target);

                    // A ball going through pulls the RPM down. Time how long it takes to come back.
                    if (!inDip && avg < target - Shooter.RPM_TOLERANCE) {
                        inDip = true;
                        dipLowest = avg;
                        dipTimer.reset();
                    }
                    if (inDip) {
                        dipLowest = Math.min(dipLowest, avg);
                        if (atSpeed) {
                            inDip = false;
                            lastDip = target - dipLowest;
                            lastRecoverySec = dipTimer.seconds();
                        }
                    }
                }
            }

            // ---------- telemetry ----------
            telemetry.addData("Flywheels", on ? "ON (A to stop)" : "off (A to spin up)");
            telemetry.addData("Target RPM", "%.0f", target);
            telemetry.addData("Left / Right RPM", "%.0f / %.0f", l, r);
            telemetry.addData("Left / Right power", "%.2f / %.2f", leftPower, rightPower);
            telemetry.addData("At speed", atSpeed);
            telemetry.addLine();
            telemetry.addData("Spin-up time (s)", riseSec < 0 ? "-" : String.format("%.2f", riseSec));
            telemetry.addData("Overshoot (rpm)", "%.0f", overshoot);
            telemetry.addData("Last dip (rpm)", "%.0f", lastDip);
            telemetry.addData("Recovery (s)", lastRecoverySec < 0 ? "-" : String.format("%.2f", lastRecoverySec));
            telemetry.addLine();
            telemetry.addLine("Gains now (edit in Panels > Shooter). Paste into Shooter.java:");
            telemetry.addLine("FLYWHEEL_KP = " + fmt(Shooter.FLYWHEEL_KP) + ";");
            telemetry.addLine("FLYWHEEL_KI = " + fmt(Shooter.FLYWHEEL_KI) + ";");
            telemetry.addLine("FLYWHEEL_KD = " + fmt(Shooter.FLYWHEEL_KD) + ";");
            telemetry.addLine("FLYWHEEL_KF = " + fmt(Shooter.FLYWHEEL_KF) + ";");
            telemetry.addLine();
            // Plain numbers for the Panels graph.
            telemetry.addData("target", on ? target : 0);
            telemetry.addData("left", l);
            telemetry.addData("right", r);
            telemetry.update();
        }
        left.setPower(0);
        right.setPower(0);
    }

    private static String fmt(double v) {
        return String.format("%.6g", v);
    }
}
