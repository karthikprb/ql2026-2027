package org.firstinspires.ftc.teamcode.OpModes.ComponentTest.shooter;

import com.bylazar.gamepad.PanelsGamepad;
import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Components.Intake;
import org.firstinspires.ftc.teamcode.Components.Shooter;
import org.firstinspires.ftc.teamcode.follower.api.PoseFactory;
import org.firstinspires.ftc.teamcode.follower.drivetrain.DrivePowers;
import org.firstinspires.ftc.teamcode.follower.hardware.Mecanum;
import org.firstinspires.ftc.teamcode.follower.hardware.PinpointLocalizer;
import org.firstinspires.ftc.teamcode.follower.math.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.ArrayList;
import java.util.List;

/**
 * COMBINED test, run this LAST. It needs RPM, hood and turret working together plus the
 * robot's field position. Finds:
 *   RPM_BASE, RPM_PER_INCH, HOOD_BASE, HOOD_PER_INCH   (the distance -> shot formulas)
 *   a correction for TURRET_FORWARD_DEG                 (if every shot needs the same turret trim)
 *
 * Paste the numbers from tests 1-5 into Shooter.java and rebuild before running this,
 * because it uses the Shooter class as-is. Also measure the GOAL x / y in Shooter.java,
 * since the distance and turret aim are worked out from them.
 *
 * Put the robot at START_X / START_Y / START_HEADING_DEG below (or set them in Panels) with the turret at TURRET_START_DEG, then INIT.
 * During init: X = blue, B = red.
 *
 * Drive to a spot, adjust RPM and hood until shots go in reliably, then save the point.
 * Do this at 4 or more distances spread across where you will shoot from in a match.
 *
 * Gamepad 1 (driver):
 *   Sticks         = drive
 *   Right bumper   = half speed
 *   A              = reset pose to the start pose (only if the robot is back on that spot)
 *
 * Gamepad 2 (shooter):
 *   A              = flywheels on / off
 *   Dpad up / down = RPM +/- 50
 *   Dpad right/left= hood +/- 0.01
 *   Bumpers        = turret trim +/- 1 deg (right bumper = turn right)
 *   Right trigger  = hold to feed (runs the intake; change feed() if you feed differently)
 *   Y              = SAVE this point (distance, RPM, hood, turret trim)
 *   X              = delete the last saved point
 *   B              = load RPM and hood from the current fit for this distance
 */
@Configurable
@TeleOp(name = "Shooter 6: Shot Table", group = "Shooter")
public class ShotTableTest extends LinearOpMode {

    // Where the robot is when you press INIT. Same as the blue auto start. Editable in Panels.
    private static final PoseFactory P = PoseFactory.degrees();
    public static double START_X = 85;
    public static double START_Y = 132;
    public static double START_HEADING_DEG = 270;

    // Gamepad 2 step sizes, editable in Panels.
    public static double RPM_STEP = 50;
    public static double HOOD_STEP = 0.01;
    public static double TURRET_TRIM_STEP_DEG = 1;

    private static final int MAX_POINTS_SHOWN = 8;

    /** One saved shot. */
    private static class Point {
        final double distance, rpm, hood, turretTrim;

        Point(double distance, double rpm, double hood, double turretTrim) {
            this.distance = distance;
            this.rpm = rpm;
            this.hood = hood;
            this.turretTrim = turretTrim;
        }
    }

    private Intake intake;

    @Override
    public void runOpMode() {
        // Every telemetry line goes to both the Driver Station and Panels.
        telemetry = new JoinedTelemetry(PanelsTelemetry.INSTANCE.getFtcTelemetry(), telemetry);
        // Gamepads work from the Driver Station or a controller plugged into the Panels computer.
        PanelsPad pad1 = new PanelsPad(gamepad1, PanelsGamepad.INSTANCE.getFirstManager());
        PanelsPad pad2 = new PanelsPad(gamepad2, PanelsGamepad.INSTANCE.getSecondManager());

        Shooter shooter = new Shooter(hardwareMap);
        intake = new Intake(hardwareMap);
        PinpointLocalizer localizer = new PinpointLocalizer(hardwareMap, Constants.localizerConfig);
        Mecanum drive = new Mecanum(hardwareMap, Constants.drivetrainConfig);

        localizer.setPose(startPose());

        while (opModeInInit()) {
            pad1.update();
            pad2.update();
            if (pad1.xWasPressed() || pad2.xWasPressed()) shooter.setAlliance(Shooter.Alliance.BLUE);
            if (pad1.bWasPressed() || pad2.bWasPressed()) shooter.setAlliance(Shooter.Alliance.RED);
            telemetry.addData("Alliance", "%s   (X = blue, B = red)", shooter.getAlliance());
            telemetry.addData("Start pose", "%.1f, %.1f, %.0f deg",
                    START_X, START_Y, START_HEADING_DEG);
            telemetry.addData("Turret must be at", "%.0f deg (TURRET_START_DEG)", Shooter.TURRET_START_DEG);
            telemetry.update();
        }
        if (isStopRequested()) return;

        localizer.setPose(startPose());

        List<Point> points = new ArrayList<>();
        boolean flywheelOn = false;
        double rpm = Shooter.rpmForDistance(0);
        double hood = Shooter.hoodForDistance(0);
        double turretTrim = 0;
        boolean loadedStart = false;

        while (opModeIsActive()) {
            pad1.update();
            pad2.update();
            localizer.update();
            Pose pose = localizer.pose();

            // ---------- driver ----------
            if (pad1.aWasPressed()) localizer.setPose(startPose());
            double scale = pad1.raw().right_bumper ? 0.5 : 1.0;
            drive.drive(new DrivePowers(
                    -pad1.raw().left_stick_y * scale,
                    -pad1.raw().left_stick_x * scale,
                    -pad1.raw().right_stick_x * scale), true);

            // ---------- aim the turret, then use our own RPM / hood ----------
            shooter.aimAt(pose);
            double distance = shooter.getLastDistanceToGoal();
            if (!loadedStart) {
                // First loop: start from Shooter's current formula at this distance.
                rpm = Shooter.rpmForDistance(distance);
                hood = Shooter.hoodForDistance(distance);
                loadedStart = true;
            }

            if (pad2.aWasPressed()) flywheelOn = !flywheelOn;
            if (pad2.dpadUpWasPressed()) rpm += RPM_STEP;
            if (pad2.dpadDownWasPressed()) rpm -= RPM_STEP;
            if (pad2.dpadRightWasPressed()) hood += HOOD_STEP;
            if (pad2.dpadLeftWasPressed()) hood -= HOOD_STEP;
            if (pad2.rightBumperWasPressed()) turretTrim += TURRET_TRIM_STEP_DEG;
            if (pad2.leftBumperWasPressed()) turretTrim -= TURRET_TRIM_STEP_DEG;
            rpm = Range.clip(rpm, 0, Shooter.FLYWHEEL_MAX_RPM);
            hood = Range.clip(hood, Shooter.HOOD_MIN, Shooter.HOOD_MAX);

            if (pad2.yWasPressed()) points.add(new Point(distance, rpm, hood, turretTrim));
            if (pad2.xWasPressed() && !points.isEmpty()) points.remove(points.size() - 1);

            double[] rpmFit = fit(points, true);
            double[] hoodFit = fit(points, false);
            if (pad2.bWasPressed()) {
                if (rpmFit != null) rpm = rpmFit[0] + rpmFit[1] * distance;
                if (hoodFit != null) hood = hoodFit[0] + hoodFit[1] * distance;
                rpm = Range.clip(rpm, 0, Shooter.FLYWHEEL_MAX_RPM);
                hood = Range.clip(hood, Shooter.HOOD_MIN, Shooter.HOOD_MAX);
            }

            shooter.setTurretDegrees(shooter.commandedTurretDeg() + turretTrim);
            shooter.setTargetRpm(flywheelOn ? rpm : 0);
            shooter.setHoodPosition(hood);
            shooter.update();

            feed(pad2.raw().right_trigger > 0.3);

            // ---------- telemetry ----------
            telemetry.addData("Goal", shooter.getActiveGoal().name);
            telemetry.addData("Robot", "%.1f, %.1f, %.0f deg",
                    pose.x(), pose.y(), Math.toDegrees(pose.heading()));
            telemetry.addData("Distance to goal (in)", "%.1f", distance);
            telemetry.addLine();
            telemetry.addData("RPM target", "%.0f  %s", rpm, flywheelOn ? "ON" : "off (gp2 A)");
            telemetry.addData("RPM left / right", "%.0f / %.0f", shooter.getLeftRpm(), shooter.getRightRpm());
            telemetry.addData("Hood", "%.3f", hood);
            telemetry.addData("Turret trim (deg)", "%.0f", turretTrim);
            telemetry.addData("Turret target / now", "%.1f / %.1f",
                    shooter.commandedTurretDeg(), shooter.getTurretDegrees());
            telemetry.addData("Ready to shoot", shooter.readyToShoot());
            telemetry.addLine();

            telemetry.addData("Saved points", points.size());
            for (int i = Math.max(0, points.size() - MAX_POINTS_SHOWN); i < points.size(); i++) {
                Point p = points.get(i);
                telemetry.addLine(String.format("  %d: %.0f in  ->  %.0f rpm, hood %.3f, trim %.0f",
                        i + 1, p.distance, p.rpm, p.hood, p.turretTrim));
            }

            if (rpmFit != null && hoodFit != null) {
                telemetry.addLine();
                telemetry.addLine("Paste into Shooter.java:");
                telemetry.addLine(String.format("RPM_BASE = %.1f;", rpmFit[0]));
                telemetry.addLine(String.format("RPM_PER_INCH = %.3f;", rpmFit[1]));
                telemetry.addLine(String.format("HOOD_BASE = %.4f;", hoodFit[0]));
                telemetry.addLine(String.format("HOOD_PER_INCH = %.5f;", hoodFit[1]));

                double avgTrim = 0;
                for (Point p : points) avgTrim += p.turretTrim;
                avgTrim /= points.size();
                if (Math.abs(avgTrim) >= 1) {
                    telemetry.addLine(String.format("TURRET_FORWARD_DEG = %.1f;   (shots needed %.1f deg trim on average)",
                            Shooter.TURRET_FORWARD_DEG + avgTrim, avgTrim));
                }
                telemetry.addData("Worst miss of the fit", "%.0f rpm, %.3f hood",
                        worstMiss(points, rpmFit, true), worstMiss(points, hoodFit, false));
                telemetry.addLine("If the worst miss is big, a straight line doesn't fit.");
                telemetry.addLine("Then save more points or switch Shooter to a lookup table.");
            } else {
                telemetry.addLine("Save points at 2+ different distances to get a fit.");
            }
            telemetry.update();
        }
        shooter.stop();
        shooter.update();
        intake.stop();
        drive.stop();
    }

    /** The start pose from START_X / START_Y / START_HEADING_DEG, read fresh so Panels edits count. */
    private static Pose startPose() {
        return P.of(START_X, START_Y, START_HEADING_DEG);
    }

    /**
     * How the ball gets from the intake into the flywheels. Right now it runs the intake.
     * If your robot has a separate feeder or gate, change this.
     */
    private void feed(boolean on) {
        intake.setState(on ? Intake.State.INTAKE : Intake.State.STOW);
    }

    /** Straight-line fit (least squares) of RPM or hood against distance: {base, perInch}, or null. */
    private static double[] fit(List<Point> points, boolean rpm) {
        int n = points.size();
        if (n < 2) return null;
        double mx = 0, my = 0;
        for (Point p : points) {
            mx += p.distance;
            my += rpm ? p.rpm : p.hood;
        }
        mx /= n;
        my /= n;
        double sxx = 0, sxy = 0;
        for (Point p : points) {
            double dx = p.distance - mx;
            sxx += dx * dx;
            sxy += dx * ((rpm ? p.rpm : p.hood) - my);
        }
        if (sxx < 1.0) return null;   // all points at about the same distance
        double perInch = sxy / sxx;
        return new double[]{my - perInch * mx, perInch};
    }

    private static double worstMiss(List<Point> points, double[] line, boolean rpm) {
        double worst = 0;
        for (Point p : points) {
            double predicted = line[0] + line[1] * p.distance;
            worst = Math.max(worst, Math.abs((rpm ? p.rpm : p.hood) - predicted));
        }
        return worst;
    }
}
