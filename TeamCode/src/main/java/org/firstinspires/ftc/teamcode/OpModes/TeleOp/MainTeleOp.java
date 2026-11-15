package org.firstinspires.ftc.teamcode.OpModes.TeleOp;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Components.Drive;
import org.firstinspires.ftc.teamcode.Components.Intake;
import org.firstinspires.ftc.teamcode.Components.Shooter;
import org.firstinspires.ftc.teamcode.follower.api.PoseFactory;
import org.firstinspires.ftc.teamcode.follower.follower.Follower;
import org.firstinspires.ftc.teamcode.follower.math.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.Field;
import org.firstinspires.ftc.teamcode.pedroPathing.Field.Alliance;

/**
 * The match TeleOp: driving, intake, shooter and turret auto-tracking. Pick the alliance on the
 * Driver Station: "Main TeleOp BLUE" or "Main TeleOp RED" (MainTeleOpBlue / MainTeleOpRed, which
 * only say which alliance this is). Everything else is in this class.
 *
 * ---------- Before INIT ----------
 *   Put the robot at the start pose (START_X / START_Y, red is the mirror image), FACING its
 *   alliance's FAR HIVE cell (heading 270 = facing the audience), with the turret pointing
 *   straight forward. The odometry and the turret both count from there.
 *
 * ---------- Gamepad 1: everything needed to play ----------
 *   Left stick    = drive / strafe        Right stick X = turn
 *   Right trigger = hold for slow mode    Options       = field-centric on / off (forward = toward the audience)
 *   A = intake on   B = intake off (stow)   X = outtake
 *   Right bumper = shooter ON (Shooter.SHOOTER_RPM)   Left bumper = shooter OFF
 *   Y = hood up / down (starts down)
 *   Dpad down = reset heading to 270 (keeps x / y). Square up facing the far cell first.
 *   Dpad up   = turret tracking on / off (off = turret straight forward). Starts on.
 *
 * ---------- Gamepad 2: fine tuning only, never needed to play ----------
 *   Dpad up / down = RPM trim +/- Shooter.RPM_STEP      Dpad right / left = turret trim +/- Shooter.TURRET_STEP_DEG
 *   Options        = clear all trims
 *
 * ---------- Turret tracking ----------
 *   Every loop the turret is pointed at the alliance's HIVE cell on the robot's end of the field,
 *   from the odometry pose (Shooter.trackGoal):
 *     robot y > Shooter.GOAL_SWITCH_Y  ->  FAR cell  (robot normally facing 270)
 *     robot y < Shooter.GOAL_SWITCH_Y  ->  NEAR cell (robot flipped around, facing 90)
 *   The turret can only turn TURRET_MIN_DEG..TURRET_MAX_DEG. If the robot is turned so far that
 *   the cell is outside that range, the turret waits at the nearest end ("AT LIMIT" in telemetry)
 *   and picks the cell up again as soon as the robot turns back toward it.
 *
 * All the gamepad code lives in Drive / Intake / Shooter: change controls there.
 */
@Configurable
public abstract class MainTeleOp extends LinearOpMode {

    // Where the robot is at START, in BLUE numbers. Red uses the mirror image (x -> 144 - x).
    // Editable in Panels before START.
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
        Drive drive = new Drive(follower, startPose(alliance).heading());
        Intake intake = new Intake(hardwareMap);
        Shooter shooter = new Shooter(hardwareMap);   // with the turret
        shooter.setAlliance(alliance);

        while (opModeInInit()) {
            Pose start = startPose(alliance);
            telemetry.addData("Alliance", alliance);
            telemetry.addLine("Robot at the start pose FACING the far cell, turret pointing FORWARD.");
            telemetry.addData("Start pose", "%.1f, %.1f, %.0f deg", start.x(), start.y(), Math.toDegrees(start.heading()));
            telemetry.addLine("GP1: sticks drive, RT slow, Options field-centric, dpad down = heading 270");
            telemetry.addLine("GP1: A intake, B stow, X outtake, Y hood, RB shooter on, LB off, dpad up = tracking");
            telemetry.addLine("GP2 (tuning only): dpad up/down RPM trim, dpad L/R turret trim, Options clear");
            telemetry.update();
        }
        if (isStopRequested()) return;

        follower.setPose(startPose(alliance));
        follower.manual(0, 0, 0);

        while (opModeIsActive()) {
            follower.update();

            drive.handleGamepad(gamepad1);        // may reset the heading, so read the pose after it
            intake.handleGamepad(gamepad1);
            shooter.handleGamepad(gamepad1);
            shooter.handleTrimGamepad(gamepad2);

            Pose pose = follower.pose();
            shooter.trackGoal(pose);              // turret follows the cell on this end of the field
            shooter.update();

            // ---------------- telemetry ----------------
            telemetry.addData("Alliance", alliance);
            telemetry.addData("Drive", (drive.isFieldCentric() ? "FIELD-centric" : "ROBOT-centric")
                    + (drive.isSlow() ? "  SLOW" : "") + "  (Options to switch)");
            telemetry.addData("Robot", "%.1f, %.1f, %.1f deg   (dpad down = heading 270)",
                    pose.x(), pose.y(), Math.toDegrees(pose.heading()));
            telemetry.addLine();
            telemetry.addData("Tracking", shooter.isTracking()
                    ? "ON  -> " + shooter.getActiveGoal().name + (shooter.goalInTurretRange() ? "" : "   AT LIMIT")
                    : "OFF, turret forward   (dpad up)");
            telemetry.addData("Turret target / now (deg)", "%.1f / %.1f",
                    shooter.commandedTurretDeg(), shooter.getTurretDegrees());
            telemetry.addData("Distance to cell (in)", "%.1f", shooter.getLastDistanceToGoal());
            telemetry.addLine();
            telemetry.addData("Intake", intake.getState());
            telemetry.addData("Shooter", shooter.isShooterOn() ? "ON" : "off");
            telemetry.addData("Target RPM (with trim)", "%.0f", shooter.commandedRpm());
            telemetry.addData("Left / Right RPM", "%.0f / %.0f", shooter.getLeftRpm(), shooter.getRightRpm());
            telemetry.addData("Ready to shoot", shooter.readyToShoot());
            telemetry.addData("Hood", "%s  %.3f  (Y to toggle)", shooter.isHoodUp() ? "UP" : "down", shooter.commandedHood());
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

    /** The start pose for this alliance, read fresh so Panels edits before START count. */
    private static Pose startPose(Alliance alliance) {
        return Field.forAlliance(alliance, PoseFactory.degrees().of(START_X, START_Y, START_HEADING_DEG));
    }
}
