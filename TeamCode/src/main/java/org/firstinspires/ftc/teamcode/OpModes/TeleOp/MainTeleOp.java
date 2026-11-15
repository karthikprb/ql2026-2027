package org.firstinspires.ftc.teamcode.OpModes.TeleOp;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Components.Drive;
import org.firstinspires.ftc.teamcode.Components.Intake;
import org.firstinspires.ftc.teamcode.Components.Shooter;
import org.firstinspires.ftc.teamcode.Components.ShotSequence;
import org.firstinspires.ftc.teamcode.follower.api.PoseFactory;
import org.firstinspires.ftc.teamcode.follower.follower.Follower;
import org.firstinspires.ftc.teamcode.follower.math.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.Field;
import org.firstinspires.ftc.teamcode.pedroPathing.Field.Alliance;
import org.firstinspires.ftc.teamcode.pedroPathing.PoseStore;

/**
 * The match TeleOp: driving, intake, shooter and turret auto-tracking. Pick the alliance on the
 * Driver Station: "Main TeleOp BLUE" or "Main TeleOp RED" (MainTeleOpBlue / MainTeleOpRed, which
 * only say which alliance this is). Everything else is in this class.
 *
 * ---------- Before INIT ----------
 *   After an auto: leave the robot where the auto ended. The auto saved its last pose
 *   (PoseStore) and this TeleOp starts the odometry from it, so nothing to line up.
 *   Without an auto (practice): put the robot at the default start pose, FACING its alliance's
 *   HIVE cell. Blue: (START_X, START_Y) = (85, 120) at the far wall, heading 270 (toward the
 *   audience). Red: the field turned 180 deg, (59, 24) at the audience wall, heading 90 (away
 *   from it). Init telemetry says which pose it will use; X during init switches between them.
 *   Either way the turret must be pointing straight forward at INIT: it counts from there.
 *
 * ---------- Gamepad 1: everything needed to play ----------
 *   Left stick    = drive / strafe        Right stick X = turn
 *   Right trigger = hold for slow mode    Options       = field-centric on / off (forward = the start heading, toward the cell)
 *   A = intake toggle: off -> on at full power, on -> off      B = intake off (stow)   X = outtake
 *   Dpad left = intake on at 0.3 power   Dpad right = intake on at 0.6 power
 *   Right bumper = shooter ON (Shooter.SHOOTER_RPM)   Left bumper = shooter OFF
 *   Y = SHOT SEQUENCE: hood up -> flywheels on -> at speed, intake feeds for 3 s -> hood down,
 *       intake off, flywheels back to how they were. Y again (or LB) cancels. (ShotSequence)
 *   Share = hood up / down by hand (starts down)
 *   Dpad down = reset heading to the start heading (270 blue, 90 red), keeps x / y.
 *               Square the robot up facing its cell first.
 *   Dpad up   = turret tracking on / off (off = turret straight forward). Starts on.
 *
 * ---------- Gamepad 2: fine tuning only, never needed to play ----------
 *   Dpad up / down = RPM trim +/- Shooter.RPM_STEP      Dpad right / left = turret trim +/- Shooter.TURRET_STEP_DEG
 *   Options        = clear all trims
 *
 * ---------- Turret tracking ----------
 *   Every loop the turret is pointed at the alliance's HIVE cell on the robot's end of the field,
 *   from the odometry pose (Shooter.trackGoal):
 *     robot y > Shooter.GOAL_SWITCH_Y  ->  FAR cell  (blue's own half; robot facing 270)
 *     robot y < Shooter.GOAL_SWITCH_Y  ->  NEAR cell (red's own half; robot facing 90)
 *   The turret can only turn TURRET_MIN_DEG..TURRET_MAX_DEG. If the robot is turned so far that
 *   the cell is outside that range, the turret waits at the nearest end ("AT LIMIT" in telemetry)
 *   and picks the cell up again as soon as the robot turns back toward it.
 *
 * All the gamepad code lives in Drive / Intake / Shooter / ShotSequence: change controls there.
 */
@Configurable
public abstract class MainTeleOp extends LinearOpMode {

    // Default start pose, used when no auto ran first. BLUE numbers; red is the field turned
    // 180 deg (x -> 144 - x, y -> 144 - y, heading + 180). Editable in Panels before START.
    public static double START_X = 85;
    public static double START_Y = 120;
    public static double START_HEADING_DEG = 270;   // facing the audience = straight at the far cell

    /** BLUE or RED: the only thing the two TeleOp classes decide. */
    protected abstract Alliance alliance();

    @Override
    public void runOpMode() {
        // Every telemetry line goes to both the Driver Station and Panels.
        telemetry = new JoinedTelemetry(PanelsTelemetry.INSTANCE.getFtcTelemetry(), telemetry);

        Alliance alliance = alliance();
        Follower follower = Constants.create(hardwareMap);
        // homeHeading (field-centric forward, dpad down reset) is the alliance's start heading
        // (270 blue, 90 red), not the auto's end heading.
        Drive drive = new Drive(follower, defaultStartPose(alliance).heading());
        long homeDeg = Math.round(Math.toDegrees(drive.getHomeHeading()));
        Intake intake = new Intake(hardwareMap);
        Shooter shooter = new Shooter(hardwareMap);   // with the turret
        shooter.setAlliance(alliance);
        ShotSequence shot = new ShotSequence(shooter, intake);

        // Start from where the auto left the robot, if an auto for this alliance just ran.
        boolean autoPoseAvailable = PoseStore.hasPoseFor(alliance);
        boolean useAutoPose = autoPoseAvailable;

        while (opModeInInit()) {
            if (autoPoseAvailable && gamepad1.xWasPressed()) useAutoPose = !useAutoPose;
            Pose start = useAutoPose ? PoseStore.pose() : defaultStartPose(alliance);

            telemetry.addData("Alliance", alliance);
            if (useAutoPose) {
                telemetry.addData("Start pose", "FROM AUTO (%.0f s ago): %.1f, %.1f, %.0f deg",
                        PoseStore.ageSeconds(), start.x(), start.y(), Math.toDegrees(start.heading()));
                telemetry.addLine("Leave the robot where the auto ended. X = use the default pose instead.");
            } else {
                telemetry.addData("Start pose", "DEFAULT: %.1f, %.1f, %.0f deg", start.x(), start.y(), Math.toDegrees(start.heading()));
                telemetry.addLine(autoPoseAvailable
                        ? "Robot at the default pose FACING its cell. X = use the auto's pose instead."
                        : "No auto pose saved. Robot at the default pose FACING its cell.");
            }
            telemetry.addLine("Turret must point straight FORWARD now.");
            telemetry.addLine("GP1: sticks drive, RT slow, Options field-centric, dpad down = heading " + homeDeg);
            telemetry.addLine("GP1: A intake on/off, dpad L/R intake 0.3/0.6, B stow, X outtake");
            telemetry.addLine("GP1: Y = shot sequence, Share = hood, RB shooter on, LB off, dpad up = tracking");
            telemetry.addLine("GP2 (tuning only): dpad up/down RPM trim, dpad L/R turret trim, Options clear");
            telemetry.update();
        }
        if (isStopRequested()) return;

        follower.setPose(useAutoPose ? PoseStore.pose() : defaultStartPose(alliance));
        PoseStore.clear();   // used once: the next TeleOp run starts from the default unless an auto runs again
        follower.manual(0, 0, 0);

        while (opModeIsActive()) {
            follower.update();

            drive.handleGamepad(gamepad1);        // may reset the heading, so read the pose after it
            intake.handleGamepad(gamepad1);
            shooter.handleGamepad(gamepad1);
            shooter.handleTrimGamepad(gamepad2);
            shot.handleGamepad(gamepad1);         // Y shot sequence; runs after the others so it has the last word

            Pose pose = follower.pose();
            shooter.trackGoal(pose);              // turret follows the cell on this end of the field
            shooter.update();

            // ---------------- telemetry ----------------
            telemetry.addData("Alliance", alliance);
            telemetry.addData("Drive", (drive.isFieldCentric() ? "FIELD-centric" : "ROBOT-centric")
                    + (drive.isSlow() ? "  SLOW" : "") + "  (Options to switch)");
            telemetry.addData("Robot", "%.1f, %.1f, %.1f deg   (dpad down = heading %d)",
                    pose.x(), pose.y(), Math.toDegrees(pose.heading()), homeDeg);
            telemetry.addLine();
            telemetry.addData("Tracking", shooter.isTracking()
                    ? "ON  -> " + shooter.getActiveGoal().name + (shooter.goalInTurretRange() ? "" : "   AT LIMIT")
                    : "OFF, turret forward   (dpad up)");
            telemetry.addData("Turret target / now (deg)", "%.1f / %.1f",
                    shooter.commandedTurretDeg(), shooter.getTurretDegrees());
            telemetry.addData("Distance to cell (in)", "%.1f", shooter.getLastDistanceToGoal());
            telemetry.addLine();
            telemetry.addData("Intake", "%s%s", intake.getState(),
                    intake.getState() == Intake.State.INTAKE ? String.format("  power %.1f", intake.getIntakePower()) : "");
            telemetry.addData("Shot (Y)", shot.isRunning()
                    ? String.format("%s  %.1f s", shot.getState(), shot.secondsInState()) : "idle");
            telemetry.addData("Shooter", shooter.isShooterOn() ? "ON" : "off");
            telemetry.addData("Target RPM (with trim)", "%.0f", shooter.commandedRpm());
            telemetry.addData("Left / Right RPM", "%.0f / %.0f", shooter.getLeftRpm(), shooter.getRightRpm());
            telemetry.addData("Ready to shoot", shooter.readyToShoot());
            telemetry.addData("Hood", "%s  %.3f  (Share to toggle)", shooter.isHoodUp() ? "UP" : "down", shooter.commandedHood());
            telemetry.addData("Trims RPM / turret", "%.0f / %.1f", shooter.getRpmTrim(), shooter.getTurretTrimDeg());
            // Plain numbers for the Panels graph.
            telemetry.addData("targetRpm", shooter.commandedRpm());
            telemetry.addData("leftRpm", shooter.getLeftRpm());
            telemetry.addData("rightRpm", shooter.getRightRpm());
            telemetry.addData("turretTarget", shooter.commandedTurretDeg());
            telemetry.addData("turretNow", shooter.getTurretDegrees());
            telemetry.update();
        }

        drive.stop();
        intake.stop();
        shooter.stop();
        shooter.update();
    }

    /** The default start pose for this alliance, read fresh so Panels edits before START count. */
    private static Pose defaultStartPose(Alliance alliance) {
        return Field.forAlliance(alliance, PoseFactory.degrees().of(START_X, START_Y, START_HEADING_DEG));
    }
}
