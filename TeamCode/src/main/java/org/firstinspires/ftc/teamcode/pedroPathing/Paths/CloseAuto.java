package org.firstinspires.ftc.teamcode.pedroPathing.Paths;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Components.Intake;
import org.firstinspires.ftc.teamcode.Components.Shooter;
import org.firstinspires.ftc.teamcode.follower.api.PoseFactory;
import org.firstinspires.ftc.teamcode.follower.follower.Follower;
import org.firstinspires.ftc.teamcode.follower.math.Pose;
import org.firstinspires.ftc.teamcode.follower.paths.Path;
import org.firstinspires.ftc.teamcode.follower.utils.Angle;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.Field.Alliance;
import org.firstinspires.ftc.teamcode.pedroPathing.PoseStore;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import static org.firstinspires.ftc.teamcode.follower.api.Paths.*;

/**
 * Close auto: shoots the preload at start (flywheels to 2500 RPM, then 3 s of intake feed),
 * waits 2 s, drives to firstStop, waits 3 s, then parks.
 *
 * One class for both alliances: BlueClose and RedClose only say which alliance they are, and
 * poses() holds each alliance's poses. Red is blue turned 180 degrees about the field centre
 * (see Field): blue starts at the far wall facing the audience (270), red starts at the
 * audience wall facing away (90), and both then drive diagonally to the driver's LEFT and
 * on diagonally further away to park. The red start (47, 8, 90) was measured on the field;
 * the other red poses move the same distances from it as blue's do from blue's start.
 *
 * The route is a list of legs: each one waits, drives one path, then waits again.
 * Edit the route in route(). Everything below the route runs it and should rarely need changes.
 */
public abstract class CloseAuto extends LinearOpMode {

    /** BLUE or RED: the only thing the two auto classes decide. */
    protected abstract Alliance alliance();

    // =================================================================
    //  SETTINGS
    // =================================================================
    private static final double DRIVE_POWER = 0.7;   // max power while driving along a path
    private static final double START_DELAY = 0.0;   // s after START before the first leg

    // Defaults for every leg. Override any of them per leg in route().
    private static final double WAIT_BEFORE     = 0.0;   // s, pause before driving
    private static final double WAIT_AFTER      = 0.2;   // s, pause after arriving
    private static final double TIMEOUT         = 5.0;   // s, max time to reach the end
    private static final double POS_TOL         = 1.5;   // in, close enough to the end
    private static final double HEADING_TOL     = 5.0;   // deg, close enough to the end heading
    private static final double TIMEOUT_POS_TOL = 4.0;   // in, still ok to continue at timeout

    // =================================================================
    //  POSES  (one set per alliance; the moves are the same, the field is turned 180 deg)
    //    Blue: facing 270 (toward the audience). Driver's left = +x, forward = -y.
    //    Red:  facing  90 (away from the audience). Driver's left = -x, forward = +y.
    //    firstStop = start + 25 in left, 17 in forward.  park = firstStop + 10 in left, 54 in forward.
    // =================================================================
    private final PoseFactory p = PoseFactory.degrees();

    private Pose start, firstStop, park;   // set in poses(), once the alliance is known

    private void poses() {
        if (alliance() == Alliance.BLUE) {
            start     = p.of(95, 137, 270);
            firstStop = p.of(120, 120, 270);
            park      = p.of(130, 66, 270);
        } else {
            start     = p.of(47, 8, 90);     // measured: the red spot that matches blue's start
            firstStop = p.of(22, 25, 90);
            park      = p.of(12, 79, 90);
        }
    }

    // =================================================================
    //  SHOT
    //    1. Flywheels spin up to SHOT_RPM. The intake stays off until they get there.
    //    2. Intake runs for FEED_TIME, feeding the balls into the flywheels.
    //    3. Flywheels and intake both turn off. The shot is done.
    //  SPINUP_TIMEOUT is a safety net: if the flywheels never reach SHOT_RPM (low battery,
    //  something jammed) the intake feeds anyway instead of the auto stalling forever.
    // =================================================================
    private static final double SHOT_RPM       = 2500;
    private static final double FEED_TIME      = 3.0;   // s the intake runs once at speed
    private static final double SPINUP_TIMEOUT = 3.0;   // s, feed anyway after this
    private static final double SHOT_MAX_TIME  = SPINUP_TIMEOUT + FEED_TIME + 0.5;

    private enum ShotState { IDLE, SPIN_UP, FEED, DONE }

    private Shooter shooter;
    private Intake intake;
    private ShotState shotState = ShotState.IDLE;
    private final ElapsedTime shotTimer = new ElapsedTime();

    /** Runs the shot one step. Call once per loop. */
    private void updateShot() {
        switch (shotState) {
            case SPIN_UP:
                shooter.setTargetRpm(SHOT_RPM);
                if (shooter.flywheelAtSpeed() || shotTimer.seconds() >= SPINUP_TIMEOUT) {
                    intake.setState(Intake.State.INTAKE);
                    shotTimer.reset();
                    shotState = ShotState.FEED;
                }
                break;
            case FEED:
                if (shotTimer.seconds() >= FEED_TIME) {
                    shooter.setTargetRpm(0);
                    intake.setState(Intake.State.STOW);
                    shotState = ShotState.DONE;
                }
                break;
            default:
                break;
        }
    }

    // =================================================================
    //  ACTIONS
    // =================================================================
    private enum Action { SHOOT, INTAKE, STOP_INTAKE }

    private void start(Action a) {
        switch (a) {
            case SHOOT:
                shotTimer.reset();
                shotState = ShotState.SPIN_UP;
                break;
            case INTAKE:      intake.setState(Intake.State.INTAKE); break;
            case STOP_INTAKE: intake.setState(Intake.State.STOW);   break;
        }
    }

    /** True once the action has finished, so the wait around a leg can end early. */
    private boolean finished(Action a) {
        switch (a) {
            case SHOOT:  return shotState == ShotState.DONE;
            case INTAKE: return false;   // no sensor to tell when the intake is full
            default:     return true;
        }
    }

    // =================================================================
    //  ROUTE - one entry per leg
    //    .waitBefore(s) / .waitAfter(s)   pause before driving / after arriving
    //    .before(action, maxS)            run an action before driving, until it finishes or maxS passes;
    //                                     .waitBefore(s) then counts from when it is done
    //    .at(s, action)                   start an action s seconds after the leg starts driving
    //    .onArrive(action)                start an action on arrival; the wait after ends early once it finishes
    //    .timeout(s) .posTol(in) .headingTol(deg) .timeoutPosTol(in)
    // =================================================================
    private List<Leg> route() {
        return Arrays.asList(
                new Leg("To firstStop", line(start, firstStop).constant(firstStop))
                        .before(Action.SHOOT, SHOT_MAX_TIME)   // preload shot before moving
                        .waitBefore(2.0)                       // then sit 2 s after the shot
                        .waitAfter(0.0),

                new Leg("To park", line(firstStop, park).constant(park))
                        .waitBefore(3.0)             // 3 s at firstStop before driving on

                // The route ends here: the robot stops at park.
        );
    }

    private static class Leg {
        final String name;
        final Path path;
        final Pose end;
        double waitBefore = WAIT_BEFORE, waitAfter = WAIT_AFTER, timeout = TIMEOUT;
        double posTol = POS_TOL, headingTol = Math.toRadians(HEADING_TOL), timeoutPosTol = TIMEOUT_POS_TOL;
        Action before = null, onArrive = null;
        double beforeMax = 0;
        boolean beforeStarted = false;
        double beforeDoneAt = -1;   // phase time when the before action finished, -1 until then
        final List<Timed> timed = new ArrayList<>();

        Leg(String name, Path path) {
            this.name = name;
            this.path = path.with(Constants.foresightConfig.maxDrivePower.at(DRIVE_POWER));
            this.end = path.endPose();
        }

        Leg waitBefore(double s)       { waitBefore = s; return this; }
        Leg waitAfter(double s)        { waitAfter = s; return this; }
        Leg timeout(double s)          { timeout = s; return this; }
        Leg posTol(double in)          { posTol = in; return this; }
        Leg headingTol(double deg)     { headingTol = Math.toRadians(deg); return this; }
        Leg timeoutPosTol(double in)   { timeoutPosTol = in; return this; }
        Leg before(Action a, double maxS) { before = a; beforeMax = maxS; return this; }
        Leg onArrive(Action a)         { onArrive = a; return this; }
        Leg at(double s, Action a)     { timed.add(new Timed(s, a)); return this; }
    }

    private static class Timed {
        final double time;
        final Action action;
        boolean started = false;

        Timed(double time, Action action) {
            this.time = time;
            this.action = action;
        }
    }

    // =================================================================
    //  RUNNER
    // =================================================================
    private enum Phase { WAIT_BEFORE, DRIVE, WAIT_AFTER, DONE }

    private Phase phase = Phase.WAIT_BEFORE;
    private int legIndex = 0;
    private final ElapsedTime phaseTimer = new ElapsedTime();
    private final ElapsedTime driveTimer = new ElapsedTime();   // timed actions count from the start of driving
    private String lastArrival = "-";
    private String failedLeg = "None";

    @Override
    public void runOpMode() throws InterruptedException {
        poses();
        Follower follower = Constants.create(hardwareMap);
        follower.setPose(start);
        intake = new Intake(hardwareMap);            // starts stowed
        shooter = new Shooter(hardwareMap, false);   // no turret: shoots straight ahead from start
        TelemetryManager telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        List<Leg> route = route();

        telemetry.addData("Alliance", alliance());
        telemetry.addData("Start pose", String.format("%.1f, %.1f, %.0f deg", start.x(), start.y(), Math.toDegrees(start.heading())));
        telemetry.update();
        waitForStart();
        if (isStopRequested()) return;
        phaseTimer.reset();

        while (opModeIsActive()) {
            follower.update();
            Pose pose = follower.pose();
            PoseStore.save(pose, alliance());   // so the TeleOp can carry on from wherever this ends
            Leg leg = phase == Phase.DONE ? null : route.get(legIndex);
            double t = phaseTimer.seconds();

            if (phase == Phase.DRIVE || phase == Phase.WAIT_AFTER) {
                for (Timed timed : leg.timed) {
                    if (!timed.started && driveTimer.seconds() >= timed.time) {
                        start(timed.action);
                        timed.started = true;
                    }
                }
            }

            switch (phase) {
                case WAIT_BEFORE: {
                    double tw = t - (legIndex == 0 ? START_DELAY : 0);   // START_DELAY holds off the whole first leg
                    if (tw < 0) break;
                    if (leg.before != null && !leg.beforeStarted) {
                        start(leg.before);
                        leg.beforeStarted = true;
                    }
                    if (leg.beforeDoneAt < 0
                            && (leg.before == null || finished(leg.before) || tw >= leg.beforeMax)) {
                        leg.beforeDoneAt = tw;
                    }
                    if (leg.beforeDoneAt >= 0 && tw >= leg.beforeDoneAt + leg.waitBefore) {
                        follower.follow(leg.path);
                        driveTimer.reset();
                        setPhase(Phase.DRIVE);
                    }
                    break;
                }

                case DRIVE: {
                    double dist = pose.distance(leg.end);
                    double headErr = Math.abs(Angle.error(pose.heading(), leg.end.heading()));
                    if (dist <= leg.posTol && headErr <= leg.headingTol) {
                        arrive(leg, "close enough");
                    } else if (t >= leg.timeout && dist <= leg.timeoutPosTol) {
                        arrive(leg, "timeout, near end");
                    } else if (t >= leg.timeout) {
                        failedLeg = leg.name;   // too far off to trust the rest of the route
                        setPhase(Phase.DONE);
                    }
                    break;
                }

                case WAIT_AFTER:
                    if (t >= leg.waitAfter || (leg.onArrive != null && finished(leg.onArrive))) {
                        legIndex++;
                        setPhase(legIndex < route.size() ? Phase.WAIT_BEFORE : Phase.DONE);
                    }
                    break;

                case DONE:
                    follower.manual(0, 0, 0);
                    break;
            }

            updateShot();
            shooter.update();   // runs the flywheel PID, needed every loop

            telemetryM.addData("Alliance", alliance());
            telemetryM.addData("Shot", shotState);
            telemetryM.addData("Target RPM", shooter.commandedRpm());
            telemetryM.addData("Left / right RPM", String.format("%.0f / %.0f", shooter.getLeftRpm(), shooter.getRightRpm()));
            telemetryM.addData("Intake", intake.getState());
            telemetryM.addData("X", pose.x());
            telemetryM.addData("Y", pose.y());
            telemetryM.addData("Heading", Math.toDegrees(pose.heading()));
            telemetryM.addData("Leg", leg == null ? "DONE" : (legIndex + 1) + "/" + route.size() + " " + leg.name);
            telemetryM.addData("Phase", phase);
            telemetryM.addData("Phase time (s)", t);
            telemetryM.addData("Distance to end", leg == null ? 0 : pose.distance(leg.end));
            telemetryM.addData("Last arrival", lastArrival);
            telemetryM.addData("Failed leg", failedLeg);
            telemetryM.update(telemetry);
        }

        shooter.stop();
        shooter.update();
        intake.stop();
    }

    private void arrive(Leg leg, String reason) {
        lastArrival = leg.name + ": " + reason;
        if (leg.onArrive != null) start(leg.onArrive);
        setPhase(Phase.WAIT_AFTER);
    }

    private void setPhase(Phase next) {
        phase = next;
        phaseTimer.reset();
    }
}
