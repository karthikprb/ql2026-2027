package org.firstinspires.ftc.teamcode.pedroPathing.Paths;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.follower.api.PoseFactory;
import org.firstinspires.ftc.teamcode.follower.follower.Follower;
import org.firstinspires.ftc.teamcode.follower.math.Pose;
import org.firstinspires.ftc.teamcode.follower.paths.Path;
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

        while (opModeIsActive()) {
            follower.update();

            Pose current = follower.pose();
            telemetryM.addData("X", current.x());
            telemetryM.addData("Y", current.y());
            telemetryM.addData("Heading", Math.toDegrees(current.heading()));
            telemetryM.addData("Mode", follower.mode());
            telemetryM.addData("Distance To End", current.distance(end));
            telemetryM.update(telemetry);
        }
    }
}
