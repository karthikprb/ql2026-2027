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

import static org.firstinspires.ftc.teamcode.follower.api.Paths.curve;

/**
 * Curves from (0, 0) to (48, 48), 48 in forward and 48 in left of where it starts.
 * The heading follows the curve, so it starts facing forward and ends facing left.
 * The robot needs that much clear space, plus room to swing out on the curve.
 */
@Autonomous(name = "Spline Test", group = "Test")
public class SplineAuto extends LinearOpMode {

    private static final double DRIVE_POWER = 0.7;

    private final PoseFactory p = PoseFactory.degrees();

    private final Pose start = p.of(0, 0, 0);
    private final Pose control = p.of(48, 0, 0);   // pulls the curve out forward before it turns left
    private final Pose end = p.of(48, 48, 90);

    @Override
    public void runOpMode() throws InterruptedException {
        Follower follower = Constants.create(hardwareMap);
        follower.setPose(start);
        TelemetryManager telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        Path spline = curve(start, control, end)
                .tangent()
                .with(Constants.foresightConfig.maxDrivePower.at(DRIVE_POWER));

        waitForStart();
        if (isStopRequested()) return;

        follower.follow(spline);

        // End-of-path numbers for tuning. The path hands off to a hold just before the end.
        Pose handoff = null;
        ElapsedTime sinceHandoff = new ElapsedTime();
        double maxPastEnd = 0;   // how far it went beyond the end, in the direction it was driving
        double settledAt = -1;   // seconds after handoff that it got within 0.5 in and 2 deg and stayed

        while (opModeIsActive()) {
            follower.update();
            Pose current = follower.pose();

            if (handoff == null && follower.mode() == Follower.Mode.HOLD) {
                handoff = current;
                sinceHandoff.reset();
            }
            if (handoff != null) {
                double past = (current.x() - end.x()) * Math.cos(end.heading())
                        + (current.y() - end.y()) * Math.sin(end.heading());
                maxPastEnd = Math.max(maxPastEnd, past);

                boolean near = current.distance(end) < 0.5
                        && Math.abs(Angle.error(current.heading(), end.heading())) < Math.toRadians(2);
                if (!near) settledAt = -1;
                else if (settledAt < 0) settledAt = sinceHandoff.seconds();
            }

            telemetryM.addData("X", current.x());
            telemetryM.addData("Y", current.y());
            telemetryM.addData("Heading", Math.toDegrees(current.heading()));
            telemetryM.addData("Mode", follower.mode());
            telemetryM.addData("Distance To End", current.distance(end));
            telemetryM.addData("Handoff at", handoff == null ? "-"
                    : String.format("%.1f, %.1f", handoff.x(), handoff.y()));
            telemetryM.addData("Max past end (in)", String.format("%.2f", maxPastEnd));
            telemetryM.addData("Settled after (s)", settledAt < 0 ? "not yet" : String.format("%.2f", settledAt));
            telemetryM.addData("Brake distance scale", Constants.BRAKE_DISTANCE_SCALE);
            telemetryM.update(telemetry);
        }
    }
}
