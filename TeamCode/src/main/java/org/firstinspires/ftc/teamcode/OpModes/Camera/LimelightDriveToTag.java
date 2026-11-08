
package org.firstinspires.ftc.teamcode.OpModes.Camera;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.follower.drivetrain.DrivePowers;
import org.firstinspires.ftc.teamcode.follower.hardware.Mecanum;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.List;

@TeleOp(name = "Limelight Drive To Tag")
public class LimelightDriveToTag extends LinearOpMode {

    private Limelight3A limelight;
    private Mecanum mecanum;
    private TelemetryManager telemetryM;

    // Configure for your field and camera.
    private static final int PIPELINE = 4;
    private static final int DESIRED_TAG_ID = -1;

    // ALL HEIGHTS MUST BE MEASURED IN METERS.
    private static final double TAG_HEIGHT_M = 1.00;
    private static final double CAMERA_HEIGHT_M = 0.25;

    // Camera pitch: positive when tilted upward.
    private static final double CAMERA_PITCH_DEG = 0.0;

    private static final double DESIRED_DISTANCE_M = 0.40;
    private static final double DISTANCE_TOLERANCE_M = 0.04;
    private static final double TX_TOLERANCE_DEG = 2.0;

    private static final double DRIVE_GAIN = 0.7;
    private static final double TURN_GAIN = 0.015;

    private static final double MAX_DRIVE = 0.25;
    private static final double MAX_TURN = 0.20;

    @Override
    public void runOpMode() throws InterruptedException {

        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        limelight = hardwareMap.get(
                Limelight3A.class, "limelight");

        mecanum = new Mecanum(hardwareMap, Constants.drivetrainConfig);

        limelight.pipelineSwitch(PIPELINE);
        limelight.start();

        telemetryM.addData("Status", "Initialized");
        telemetryM.addData("Pipeline", PIPELINE);
        telemetryM.update(telemetry);

        waitForStart();

        while (opModeIsActive()) {

            double drive = 0;
            double strafe = 0;
            double turn = 0;

            LLResult result = limelight.getLatestResult();

            LLResultTypes.FiducialResult target = null;

            if (result != null && result.isValid()) {

                List<LLResultTypes.FiducialResult> tags =
                        result.getFiducialResults();

                for (LLResultTypes.FiducialResult tag : tags) {
                    if (DESIRED_TAG_ID == -1
                            || tag.getFiducialId() == DESIRED_TAG_ID) {
                        target = tag;
                        break;
                    }
                }
            }

            if (target != null) {

                double tx = target.getTargetXDegrees();
                double ty = target.getTargetYDegrees();

                double angleDeg = CAMERA_PITCH_DEG + ty;
                double angleRad = Math.toRadians(angleDeg);

                double heightDifference =
                        TAG_HEIGHT_M - CAMERA_HEIGHT_M;

                double tanAngle = Math.tan(angleRad);

                telemetryM.addData("Tag ID",
                        target.getFiducialId());
                telemetryM.addData("TX (deg)", tx);
                telemetryM.addData("TY (deg)", ty);

                // Estimate distance from camera-to-tag height difference.
                if (Math.abs(tanAngle) > 0.001
                        && heightDifference * tanAngle > 0) {

                    double distance =
                            heightDifference / tanAngle;

                    double distanceError =
                            distance - DESIRED_DISTANCE_M;

                    telemetryM.addData(
                            "Estimated Distance (m)", distance);
                    telemetryM.addData(
                            "Distance Error (m)", distanceError);

                    if (Math.abs(tx) > TX_TOLERANCE_DEG) {
                        // Positive TX means target is to the right.
                        // Pedro's Mecanum: negative turn = clockwise.
                        turn = Range.clip(
                                -tx * TURN_GAIN,
                                -MAX_TURN, MAX_TURN);
                    }

                    if (Math.abs(distanceError)
                            > DISTANCE_TOLERANCE_M) {

                        // Pedro's Mecanum: positive = forward.
                        // Only approach when the tag is approximately centered.
                        if (Math.abs(tx) <= 10.0) {
                            double forward = Range.clip(
                                    distanceError * DRIVE_GAIN,
                                    -MAX_DRIVE, MAX_DRIVE);

                            drive = forward;
                        }
                    }

                    telemetryM.addData("Drive Command", drive);
                    telemetryM.addData("Turn Command", turn);
                    telemetryM.addData("Strafe Command", strafe);

                    telemetryM.addData(
                            "Controller",
                            Math.abs(distanceError)
                                    <= DISTANCE_TOLERANCE_M
                                    && Math.abs(tx) <= TX_TOLERANCE_DEG
                                    ? "At target" : "Approaching");

                } else {
                    telemetryM.addData(
                            "Distance", "Cannot estimate; check geometry");
                }

            } else {
                telemetryM.addData("Target", "No matching AprilTag");
            }

            mecanum.drive(new DrivePowers(drive, strafe, turn), true);

            telemetryM.update(telemetry);
            sleep(20);
        }

        limelight.stop();
        mecanum.stop();
    }
}
