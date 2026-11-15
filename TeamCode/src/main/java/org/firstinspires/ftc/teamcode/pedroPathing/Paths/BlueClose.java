package org.firstinspires.ftc.teamcode.pedroPathing.Paths;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.follower.api.PoseFactory;
import org.firstinspires.ftc.teamcode.follower.follower.Follower;
import org.firstinspires.ftc.teamcode.follower.math.Pose;
import org.firstinspires.ftc.teamcode.follower.paths.Path;
import org.firstinspires.ftc.teamcode.follower.utils.Angle;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import static org.firstinspires.ftc.teamcode.follower.api.Paths.*;

/**
 * Blue close auto: shoots the preload at start, waits, drives to firstStop, waits, then parks.
 * The route is a list of legs: each one waits, drives one path, then waits again.
 * Edit the route in route(). Everything below the route runs it and should rarely need changes.
 */
@Autonomous(name = "Blue Close")
public class BlueClose extends LinearOpMode {

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
    //  POSES
    // =================================================================
    private final PoseFactory p = PoseFactory.degrees();

    private final Pose start            = p.of(95, 144, 270);
    private final Pose firstStop        = p.of(120, 120, 270);
    private final Pose park             = p.of(130, 50, 270);

    // =================================================================
    //  ACTIONS
    // =================================================================
    private enum Action { SHOOT, INTAKE, STOP_INTAKE }

    private void start(Action a) {
        switch (a) {
            case SHOOT:       /* TODO: start shooter / feed rings */ break;
            case INTAKE:      /* TODO: intake on */                  break;
            case STOP_INTAKE: /* TODO: intake off */                 break;
        }
    }

    /** True once the action has finished, so the wait after a leg can end early. */
    private boolean finished(Action a) {
        switch (a) {
            case SHOOT:  return false;   // TODO: return shooterFinished();
            case INTAKE: return false;   // TODO: return intakeFull();
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
                        .before(Action.SHOOT, 1.5)   // preload shot before moving, up to 1.5 s
                        .waitBefore(4.0)             // then sit 4 s after the shot
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
        Follower follower = Constants.create(hardwareMap);
        follower.setPose(start);
        TelemetryManager telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        List<Leg> route = route();

        waitForStart();
        if (isStopRequested()) return;
        phaseTimer.reset();

        while (opModeIsActive()) {
            follower.update();
            Pose pose = follower.pose();
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
