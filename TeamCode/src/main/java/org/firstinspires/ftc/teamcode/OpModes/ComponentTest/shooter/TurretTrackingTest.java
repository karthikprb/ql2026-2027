package org.firstinspires.ftc.teamcode.OpModes.ComponentTest.shooter;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Components.Shooter;
import org.firstinspires.ftc.teamcode.follower.api.PoseFactory;
import org.firstinspires.ftc.teamcode.follower.drivetrain.DrivePowers;
import org.firstinspires.ftc.teamcode.follower.hardware.Mecanum;
import org.firstinspires.ftc.teamcode.follower.hardware.PinpointLocalizer;
import org.firstinspires.ftc.teamcode.follower.math.Pose;
import org.firstinspires.ftc.teamcode.follower.utils.Angle;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.Field;

/**
 * Turret auto-tracking test. Drive the robot around and the turret should keep pointing at
 * the blue goal, using only odometry (where the robot is and which way it faces).
 *
 * Setup:
 *   1. Put the robot at START_X / START_Y (default 85, 120: in front of the blue goal),
 *      FACING THE GOAL. With LINE_UP = true the start heading is worked out for you as
 *      "pointing straight at the goal" (about 268.5 deg from 85, 120). Telemetry shows it.
 *   2. Point the turret straight forward (TURRET_START_DEG) and press INIT.
 *   3. Press START. The turret should stay at forward (the robot is already lined up).
 *   4. Drive, strafe and turn. The turret should swing to keep pointing at the goal.
 *
 * The turret can only turn from TURRET_MIN_DEG to TURRET_MAX_DEG. If the goal is outside that
 * range, the turret stops at the end and telemetry shows "AT LIMIT". That's expected, not a bug.
 *
 * Gamepad 1:
 *   Sticks        = drive (robot-centric)
 *   Right bumper  = hold for half speed
 *   X             = tracking on / off (off = turret goes back to forward, to compare)
 *   A             = reset the pose to the start pose (only when the robot is back on that spot)
 *   Dpad left     = aim at BLUE_FAR   (84, 81.4)  the one in front of the default start
 *   Dpad right    = aim at BLUE_NEAR  (84, 62.6)
 *
 * The flywheels stay off the whole time.
 *
 * What to check:
 *   - Strafe right: the goal moves to the robot's left, so the turret should turn
 *     COUNTERCLOCKWISE (angle goes down toward TURRET_MIN_DEG).
 *   - Strafe left: turret turns CLOCKWISE (angle goes up toward TURRET_MAX_DEG).
 *   - Turn the robot in place: the turret should turn the opposite way by the same amount,
 *     so it keeps pointing at the same spot.
 *   - Drive back to the start and press nothing: turret should come back to about forward.
 * If the turret turns the WRONG way, check the robot's heading reading first (Odo Drive),
 * then the turret ticks / degrees in Shooter.java.
 */
@Configurable
@Disabled   // hidden from the Driver Station: test OpMode. Remove this line to use it.
@TeleOp(name = "Shooter 7: Turret Tracking", group = "Shooter")
public class TurretTrackingTest extends LinearOpMode {

    private static final PoseFactory P = PoseFactory.degrees();

    // Start position, editable in Panels. Default: in front of the blue goal.
    public static double START_X = 85;
    public static double START_Y = 120;
    /** true = start heading points straight at the goal. false = use START_HEADING_DEG. */
    public static boolean LINE_UP = true;
    public static double START_HEADING_DEG = 270;

    /** false = BLUE_FAR (84, 81.4), true = BLUE_NEAR (84, 62.6). Dpad left / right change it too. */
    public static boolean USE_NEAR_GOAL = false;

    @Override
    public void runOpMode() {
        telemetry = new JoinedTelemetry(PanelsTelemetry.INSTANCE.getFtcTelemetry(), telemetry);

        Shooter shooter = new Shooter(hardwareMap);   // with the turret
        shooter.setAlliance(Field.Alliance.BLUE);
        PinpointLocalizer localizer = new PinpointLocalizer(hardwareMap, Constants.localizerConfig);
        Mecanum drive = new Mecanum(hardwareMap, Constants.drivetrainConfig);

        while (opModeInInit()) {
            if (gamepad1.dpadLeftWasPressed()) USE_NEAR_GOAL = false;
            if (gamepad1.dpadRightWasPressed()) USE_NEAR_GOAL = true;
            Shooter.Goal goal = goal();
            Pose start = startPose(goal);
            telemetry.addLine("Robot at the start spot FACING the goal, turret pointing forward.");
            telemetry.addData("Goal", "%s  (%.1f, %.1f)   dpad L/R to change", goal.name, goal.x, goal.y);
            telemetry.addData("Start pose", "%.1f, %.1f, %.1f deg%s", start.x(), start.y(),
                    Math.toDegrees(start.heading()), LINE_UP ? "  (lined up with goal)" : "");
            telemetry.addData("Distance to goal (in)", "%.1f", Math.hypot(goal.x - start.x(), goal.y - start.y()));
            telemetry.addData("Turret range (deg)", "%.0f .. %.0f, forward %.0f",
                    Shooter.TURRET_MIN_DEG, Shooter.TURRET_MAX_DEG, Shooter.TURRET_FORWARD_DEG);
            telemetry.update();
        }
        if (isStopRequested()) return;

        localizer.setPose(startPose(goal()));
        boolean tracking = true;

        while (opModeIsActive()) {
            localizer.update();
            Pose pose = localizer.pose();

            // ---------- driver ----------
            if (gamepad1.aWasPressed()) localizer.setPose(startPose(goal()));
            if (gamepad1.xWasPressed()) tracking = !tracking;
            if (gamepad1.dpadLeftWasPressed()) USE_NEAR_GOAL = false;
            if (gamepad1.dpadRightWasPressed()) USE_NEAR_GOAL = true;

            double scale = gamepad1.right_bumper ? 0.5 : 1.0;
            drive.drive(new DrivePowers(
                    -gamepad1.left_stick_y * scale,
                    -gamepad1.left_stick_x * scale,
                    -gamepad1.right_stick_x * scale), true);

            // ---------- turret ----------
            Shooter.Goal goal = goal();
            if (tracking) {
                shooter.aimAt(pose, goal);
            } else {
                shooter.setTurretDegrees(Shooter.TURRET_FORWARD_DEG);
            }
            shooter.setTargetRpm(0);   // aimAt() sets AIM_RPM; keep the flywheels off for this test
            shooter.update();

            // ---------- the same math aimAt() does, shown step by step ----------
            double dx = goal.x - pose.x();
            double dy = goal.y - pose.y();
            double fieldAngleDeg = Math.toDegrees(Math.atan2(dy, dx));
            double relativeDeg = Math.toDegrees(Angle.normalizeSigned(Math.atan2(dy, dx) - pose.heading()));
            double wantedTurretDeg = Shooter.turretForRelativeAngle(relativeDeg);
            boolean atLimit = wantedTurretDeg < Shooter.TURRET_MIN_DEG || wantedTurretDeg > Shooter.TURRET_MAX_DEG;

            // ---------- telemetry ----------
            telemetry.addData("Tracking", tracking ? "ON  (X to turn off)" : "OFF, turret forward  (X to turn on)");
            telemetry.addData("Goal", "%s  (%.1f, %.1f)", goal.name, goal.x, goal.y);
            telemetry.addData("Robot", "%.1f, %.1f, %.1f deg",
                    pose.x(), pose.y(), Math.toDegrees(pose.heading()));
            telemetry.addData("Distance to goal (in)", "%.1f", Math.hypot(dx, dy));
            telemetry.addLine();
            telemetry.addData("Goal direction on field (deg)", "%.1f", fieldAngleDeg);
            telemetry.addData("Goal vs robot heading (deg)", "%+.1f  (%s)", relativeDeg,
                    Math.abs(relativeDeg) < 0.5 ? "straight ahead" : relativeDeg > 0 ? "to the LEFT" : "to the RIGHT");
            telemetry.addData("Turret wanted (deg)", "%.1f%s", wantedTurretDeg,
                    atLimit ? "   AT LIMIT: goal is outside the turret's range" : "");
            telemetry.addData("Turret target / now (deg)", "%.1f / %.1f",
                    shooter.commandedTurretDeg(), shooter.getTurretDegrees());
            telemetry.addData("Turret error (deg)", "%.1f", shooter.getTurretErrorDeg());
            telemetry.addData("Turret power / ticks", "%.2f / %d", shooter.getTurretPower(), shooter.getTurretTicks());
            telemetry.addData("On target", shooter.turretOnTarget());
            // Plain numbers for the Panels graph.
            telemetry.addData("turretTarget", shooter.commandedTurretDeg());
            telemetry.addData("turretNow", shooter.getTurretDegrees());
            telemetry.addData("relativeDeg", relativeDeg);
            telemetry.update();
        }
        shooter.stop();
        shooter.update();
        drive.stop();
    }

    private static Shooter.Goal goal() {
        return USE_NEAR_GOAL ? Shooter.BLUE_NEAR : Shooter.BLUE_FAR;
    }

    /** Start pose from START_X / START_Y. With LINE_UP the heading points straight at the goal. */
    private static Pose startPose(Shooter.Goal goal) {
        double headingDeg = LINE_UP
                ? Math.toDegrees(Math.atan2(goal.y - START_Y, goal.x - START_X))
                : START_HEADING_DEG;
        return P.of(START_X, START_Y, headingDeg);
    }
}
