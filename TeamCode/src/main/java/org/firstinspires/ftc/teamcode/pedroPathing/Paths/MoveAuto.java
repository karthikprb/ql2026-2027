package org.firstinspires.ftc.teamcode.pedroPathing.Paths;

import static org.firstinspires.ftc.teamcode.follower.api.Paths.line;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.follower.api.PoseFactory;
import org.firstinspires.ftc.teamcode.follower.follower.Follower;
import org.firstinspires.ftc.teamcode.follower.math.Pose;
import org.firstinspires.ftc.teamcode.follower.utils.Angle;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.PoseStore;

/**
 * Template: start at one pose, drive in a straight line to another, stop. That's all.
 *
 * The only things to edit are START and END below (field inches, heading in degrees:
 * 0 = +x, 90 = away from the audience, 270 = facing the audience). The robot turns smoothly
 * from START's heading to END's heading on the way. Copy this file and rename it for each
 * new move you want.
 */
@Autonomous(name = "Move Auto")
public class MoveAuto extends LinearOpMode {

    private static final PoseFactory p = PoseFactory.degrees();

    // ============ EDIT THESE TWO ============
    private final Pose START = p.of(95, 137, 270);
    private final Pose END   = p.of(120, 120, 270);
    // ========================================

    private static final double DRIVE_POWER     = 0.7;   // max drive power along the path
    private static final double TIMEOUT         = 6.0;   // s, stop trying after this
    private static final double POS_TOL         = 1.5;   // in, close enough to END
    private static final double HEADING_TOL_DEG = 5.0;   // deg, close enough to END's heading

    @Override
    public void runOpMode() {
        Follower follower = Constants.create(hardwareMap);
        follower.setPose(START);

        telemetry.addData("Start", "%.1f, %.1f, %.0f deg", START.x(), START.y(), Math.toDegrees(START.heading()));
        telemetry.addData("End", "%.1f, %.1f, %.0f deg", END.x(), END.y(), Math.toDegrees(END.heading()));
        telemetry.addLine("Put the robot at Start, then press START.");
        telemetry.update();
        waitForStart();
        if (isStopRequested()) return;

        follower.follow(line(START, END).linear(START, END)
                .with(Constants.foresightConfig.maxDrivePower.at(DRIVE_POWER)));
        ElapsedTime timer = new ElapsedTime();
        boolean done = false;

        while (opModeIsActive()) {
            follower.update();
            Pose pose = follower.pose();
            PoseStore.save(pose, null);   // so the TeleOp can carry on from wherever this ends
            double dist = pose.distance(END);
            double headErrDeg = Math.toDegrees(Math.abs(Angle.error(pose.heading(), END.heading())));

            if (!done && ((dist <= POS_TOL && headErrDeg <= HEADING_TOL_DEG) || timer.seconds() >= TIMEOUT)) {
                done = true;
            }
            if (done) follower.manual(0, 0, 0);   // hold still at the end

            telemetry.addData("State", done ? "DONE" : "driving");
            telemetry.addData("Robot", "%.1f, %.1f, %.1f deg", pose.x(), pose.y(), Math.toDegrees(pose.heading()));
            telemetry.addData("Distance to end (in)", "%.1f", dist);
            telemetry.addData("Heading error (deg)", "%.1f", headErrDeg);
            telemetry.addData("Time (s)", "%.1f", timer.seconds());
            telemetry.update();
        }

        follower.stop();
        follower.update();
    }
}
