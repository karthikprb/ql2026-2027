package org.firstinspires.ftc.teamcode.OpModes.TeleOp;

import static org.firstinspires.ftc.teamcode.follower.api.Paths.line;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.follower.api.PoseFactory;
import org.firstinspires.ftc.teamcode.follower.controllers.PIDController;
import org.firstinspires.ftc.teamcode.follower.drivetrain.DrivePowers;
import org.firstinspires.ftc.teamcode.follower.follower.Follower;
import org.firstinspires.ftc.teamcode.follower.follower.ManualDrive;
import org.firstinspires.ftc.teamcode.follower.math.Pose;
import org.firstinspires.ftc.teamcode.follower.math.Twist;
import org.firstinspires.ftc.teamcode.follower.paths.Path;
import org.firstinspires.ftc.teamcode.follower.utils.Angle;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

/**
 * Drive around with the sticks while the Pinpoint odometry tracks where the robot is.
 * Press A and the robot drives itself back to where it started, then lines up exactly.
 *
 * How the return works:
 *   1. PATH    - Pedro follows a straight line from where you are to home, turning smoothly
 *                to the home heading on the way. This is the smooth, fast part.
 *   2. PRECISE - Pedro hands off when the path ends. Pedro PID controllers then push the robot
 *                the last bit until it is within POS_TOL inches and HEADING_TOL_DEG degrees.
 *   3. HOME    - Within tolerance for SETTLE_TIME. The PID keeps running, so if the robot
 *                gets bumped it pushes back onto the spot.
 *
 * ---------- Before START (setting the starting position) ----------
 *   X (square)        = next preset starting position
 *   Dpad left / right = pick which number to change: X, Y or Heading
 *   Dpad up / down    = change it by 1 in or 5 deg
 *   Hold right bumper = fine steps of 0.1 in or 1 deg
 *   Put the robot on the field at that position, then press START.
 *
 * ---------- After START (gamepad 1) ----------
 *   Left stick        = drive / strafe
 *   Right stick X     = turn
 *   Right bumper      = slow mode
 *   Options           = toggle field-centric driving (forward = the way the robot faced at start)
 *   A (cross)         = return home. Press A again to cancel.
 *   Moving any stick while it is returning also cancels, so you can always take over.
 */
@TeleOp(name = "Drive + Return Home", group = "TeleOp")
public class ReturnHomeTeleOp extends LinearOpMode {

    // =================================================================
    //  STARTING POSITION PRESETS (field inches and degrees)
    //  The first one is selected when the OpMode is initialised.
    // =================================================================
    private static final PoseFactory P = PoseFactory.degrees();
    private static final String[] PRESET_NAMES = {"Origin", "Blue auto start"};
    private static final Pose[] PRESETS = {
            P.of(0, 0, 0),
            P.of(85, 132, 270),
    };

    // =================================================================
    //  DRIVING
    // =================================================================
    private static final double SLOW_MODE_SCALE = 0.4;
    private static final double TAKEOVER_STICK = 0.3;   // stick this far cancels the return

    // =================================================================
    //  RETURN HOME
    // =================================================================
    private static final double POS_TOL = 0.5;           // in, "at home" distance
    private static final double HEADING_TOL_DEG = 1.0;   // deg, "at home" heading error
    private static final double SETTLE_TIME = 0.25;      // s, must stay in tolerance this long
    private static final double RETURN_POWER = 0.7;      // max drive power while following the path
    private static final double PATH_MIN_DISTANCE = 3.0; // in, closer than this skips the path
    private static final double PATH_TIMEOUT = 8.0;      // s, give up on the path and go to PRECISE

    // Final approach, Pedro PIDControllers. D uses measured robot speed, which keeps it smooth.
    // Tuning: raise *_KP if it stops short, raise *_KD if it overshoots or wobbles.
    // *_KS is the small push needed to get the robot moving at all; raise it if the robot
    // stalls just outside tolerance, lower it if it twitches back and forth.
    private static final double TRANS_KP = 0.05;   // power per inch
    private static final double TRANS_KI = 0.0;
    private static final double TRANS_KD = 0.005;  // power per (in/s)
    private static final double TRANS_KS = 0.05;
    private static final double HEAD_KP = 1.2;     // power per radian
    private static final double HEAD_KI = 0.0;
    private static final double HEAD_KD = 0.05;    // power per (rad/s)
    private static final double HEAD_KS = 0.04;
    private static final double PRECISE_MAX_DRIVE = 0.35;
    private static final double PRECISE_MAX_TURN = 0.3;

    private enum State { DRIVER, PATH, PRECISE, HOME }

    private Follower follower;
    private TelemetryManager panels;

    private Pose home;
    private State state = State.DRIVER;
    private boolean fieldCentric = false;
    private boolean everInTolerance = false;
    private String lastResult = "-";

    private final PIDController forwardPid = new PIDController(TRANS_KP, TRANS_KI, TRANS_KD);
    private final PIDController strafePid = new PIDController(TRANS_KP, TRANS_KI, TRANS_KD);
    private final PIDController headingPid = new PIDController(HEAD_KP, HEAD_KI, HEAD_KD);

    private final ElapsedTime stateTimer = new ElapsedTime();
    private final ElapsedTime inTolTimer = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        follower = Constants.create(hardwareMap);
        panels = PanelsTelemetry.INSTANCE.getTelemetry();

        // ---------------- INIT: pick the starting position ----------------
        int preset = 0;
        int field = 0;   // 0 = X, 1 = Y, 2 = Heading
        double startX = PRESETS[0].x();
        double startY = PRESETS[0].y();
        double startHDeg = Math.toDegrees(PRESETS[0].heading());

        while (opModeInInit()) {
            if (gamepad1.xWasPressed()) {
                preset = (preset + 1) % PRESETS.length;
                startX = PRESETS[preset].x();
                startY = PRESETS[preset].y();
                startHDeg = Math.toDegrees(PRESETS[preset].heading());
            }
            if (gamepad1.dpadRightWasPressed()) field = (field + 1) % 3;
            if (gamepad1.dpadLeftWasPressed()) field = (field + 2) % 3;

            int dir = 0;
            if (gamepad1.dpadUpWasPressed()) dir = 1;
            if (gamepad1.dpadDownWasPressed()) dir = -1;
            if (dir != 0) {
                boolean fine = gamepad1.right_bumper;
                if (field == 0) startX += dir * (fine ? 0.1 : 1.0);
                else if (field == 1) startY += dir * (fine ? 0.1 : 1.0);
                else startHDeg = wrapDeg(startHDeg + dir * (fine ? 1.0 : 5.0));
            }

            String[] marks = {"  ", "  ", "  "};
            marks[field] = "> ";
            telemetry.addLine("=== SET STARTING POSITION ===");
            telemetry.addLine("X: next preset | Dpad L/R: pick | Dpad U/D: change | hold RB: fine");
            telemetry.addData("Preset", PRESET_NAMES[preset]);
            telemetry.addLine(marks[0] + "X (in): " + fmt(startX));
            telemetry.addLine(marks[1] + "Y (in): " + fmt(startY));
            telemetry.addLine(marks[2] + "Heading (deg): " + fmt(startHDeg));
            telemetry.addLine("Place the robot there, then press START.");
            telemetry.update();
        }
        if (isStopRequested()) return;

        // ---------------- START ----------------
        home = P.of(startX, startY, startHDeg);
        follower.setPose(home);
        follower.manual(0, 0, 0);

        while (opModeIsActive()) {
            follower.update();
            Pose pose = follower.pose();

            double forwardIn = -gamepad1.left_stick_y;
            double strafeIn = -gamepad1.left_stick_x;
            double turnIn = -gamepad1.right_stick_x;
            boolean sticksMoved = Math.abs(forwardIn) > TAKEOVER_STICK
                    || Math.abs(strafeIn) > TAKEOVER_STICK
                    || Math.abs(turnIn) > TAKEOVER_STICK;

            if (gamepad1.optionsWasPressed()) fieldCentric = !fieldCentric;

            // A toggles the return. Sticks always win.
            if (gamepad1.aWasPressed()) {
                if (state == State.DRIVER) startReturn(pose);
                else cancelReturn("cancelled (A)");
            } else if (state != State.DRIVER && sticksMoved) {
                cancelReturn("cancelled (sticks)");
            }

            switch (state) {
                case DRIVER: {
                    double scale = gamepad1.right_bumper ? SLOW_MODE_SCALE : 1.0;
                    DrivePowers powers = new DrivePowers(forwardIn * scale, strafeIn * scale, turnIn * scale);
                    if (fieldCentric) {
                        // Forward on the stick = the direction the robot faced at the start.
                        powers = ManualDrive.fieldCentric(powers, pose.heading(), -home.heading());
                    }
                    follower.manual(powers);
                    break;
                }

                case PATH:
                    // Pedro switches itself to HOLD when the path is finished.
                    if (follower.mode() == Follower.Mode.HOLD || stateTimer.seconds() > PATH_TIMEOUT) {
                        enterPrecise();
                    }
                    break;

                case PRECISE:
                case HOME: {
                    boolean inTol = runPrecise(pose);
                    if (!inTol) {
                        inTolTimer.reset();
                        if (state == State.HOME) state = State.PRECISE;   // got bumped
                    } else if (state == State.PRECISE && inTolTimer.seconds() >= SETTLE_TIME) {
                        state = State.HOME;
                        if (!everInTolerance) {
                            lastResult = String.format("home in %.2f s", stateTimer.seconds());
                            everInTolerance = true;
                        }
                    }
                    break;
                }
            }

            // ---------------- telemetry ----------------
            double distHome = pose.distance(home);
            double headErrDeg = Math.toDegrees(Angle.error(pose.heading(), home.heading()));

            show("State", stateLabel());
            show("Drive", fieldCentric ? "FIELD-centric (Options to switch)" : "ROBOT-centric (Options to switch)");
            show("X (in)", fmt(pose.x()));
            show("Y (in)", fmt(pose.y()));
            show("Heading (deg)", fmt(Math.toDegrees(pose.heading())));
            show("Home", fmt(home.x()) + ", " + fmt(home.y()) + ", " + fmt(Math.toDegrees(home.heading())) + " deg");
            show("Distance to home (in)", fmt(distHome));
            show("Heading error (deg)", fmt(headErrDeg));
            show("Last return", lastResult);
            telemetry.update();
            panels.update();
        }

        follower.stop();
        follower.update();
    }

    /** Pressing A: follow a smooth Pedro path home, or go straight to PRECISE if already close. */
    private void startReturn(Pose pose) {
        stateTimer.reset();
        everInTolerance = false;
        lastResult = "returning...";

        if (pose.distance(home) < PATH_MIN_DISTANCE) {
            enterPrecise();
            return;
        }

        Path toHome = line(pose, home)
                .linear(pose, home)   // turn smoothly to the home heading along the way
                .with(Constants.foresightConfig.maxDrivePower.at(RETURN_POWER));
        follower.follow(toHome);
        state = State.PATH;
    }

    private void enterPrecise() {
        forwardPid.reset();
        strafePid.reset();
        headingPid.reset();
        inTolTimer.reset();
        state = State.PRECISE;
    }

    private void cancelReturn(String why) {
        lastResult = why;
        state = State.DRIVER;
        follower.manual(0, 0, 0);
    }

    /**
     * Final approach with Pedro PIDControllers. Errors are turned into the robot's own
     * forward / left directions so they match the drive powers.
     *
     * @return true when the robot is within POS_TOL and HEADING_TOL_DEG.
     */
    private boolean runPrecise(Pose pose) {
        double dx = home.x() - pose.x();
        double dy = home.y() - pose.y();
        double h = pose.heading();
        double forwardErr = dx * Math.cos(h) + dy * Math.sin(h);
        double leftErr = -dx * Math.sin(h) + dy * Math.cos(h);
        double headErr = Angle.error(h, home.heading());

        boolean posOk = Math.hypot(dx, dy) <= POS_TOL;
        boolean headOk = Math.abs(headErr) <= Math.toRadians(HEADING_TOL_DEG);

        Twist speed = follower.twist();   // robot frame: vx forward, vy left, omega CCW

        double forward = 0, strafe = 0, turn = 0;
        if (!posOk) {
            forward = withKs(forwardPid.calculate(0, forwardErr, speed.vx), forwardErr, TRANS_KS);
            strafe = withKs(strafePid.calculate(0, leftErr, speed.vy), leftErr, TRANS_KS);
            // Keep the direction of travel when clipping so it still heads straight at home.
            double mag = Math.hypot(forward, strafe);
            if (mag > PRECISE_MAX_DRIVE) {
                forward *= PRECISE_MAX_DRIVE / mag;
                strafe *= PRECISE_MAX_DRIVE / mag;
            }
        } else {
            forwardPid.reset();
            strafePid.reset();
        }
        if (!headOk) {
            turn = withKs(headingPid.calculate(0, headErr, speed.omega), headErr, HEAD_KS);
            turn = Range.clip(turn, -PRECISE_MAX_TURN, PRECISE_MAX_TURN);
        } else {
            headingPid.reset();
        }

        follower.manual(forward, strafe, turn);
        return posOk && headOk;
    }

    /** Adds a small push in the direction of the error so tiny errors still move the robot. */
    private static double withKs(double pid, double error, double ks) {
        return pid + Math.signum(error) * ks;
    }

    private String stateLabel() {
        switch (state) {
            case PATH:    return "RETURNING - following path (A or sticks to cancel)";
            case PRECISE: return "RETURNING - lining up (A or sticks to cancel)";
            case HOME:    return "HOME - holding position (A or sticks to drive)";
            default:      return "DRIVING (A = return home)";
        }
    }

    private void show(String caption, Object value) {
        telemetry.addData(caption, value);
        panels.addData(caption, value);
    }

    private static String fmt(double v) {
        return String.format("%.2f", v);
    }

    private static double wrapDeg(double d) {
        while (d >= 360) d -= 360;
        while (d < 0) d += 360;
        return d;
    }
}
