package org.firstinspires.ftc.teamcode.OpModes.Camera;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.Components.LimelightTagLocalizer;
import org.firstinspires.ftc.teamcode.follower.hardware.PinpointLocalizer;
import org.firstinspires.ftc.teamcode.follower.math.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

/*
 * Shows the robot's offset from an AprilTag, plus the field-pose (MegaTag) values
 * for comparison.
 *
 * ONE-TIME CHECKS (robot square to the tag, about 1 m away):
 *  1. Turn the robot in place. Of "Raw yaw / pitch / roll", the one that changes
 *     about 1 degree per degree of turn is the heading. If it isn't yaw, set
 *     LimelightTagLocalizer.HEADING_AXIS to PITCH or ROLL.
 *  2. Turn left (counter-clockwise). If "Heading (deg)" goes negative,
 *     set HEADING_SIGN = -1.
 *  3. Slide the robot to its right. Pick whichever sign you want for lateral
 *     and set LATERAL_SIGN to match.
 *  4. Measure the real distance from the tag face to the robot center.
 *     "Forward" should match within a couple of cm. If it's off by a fixed
 *     amount, the camera pose in the Limelight web UI is wrong; if it's off
 *     by a ratio, the tag size is wrong.
 */
@TeleOp(name = "Limelight Tag Localization")
public class limelightTest extends LinearOpMode {

    private static final int PIPELINE = 4;
    private static final int DESIRED_TAG_ID = -1;   // -1 = closest tag in view

    @Override
    public void runOpMode() {

        TelemetryManager telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        LimelightTagLocalizer tagLocalizer =
                new LimelightTagLocalizer(hardwareMap, "limelight", PIPELINE);
        Limelight3A limelight = tagLocalizer.getLimelight();

        // Pedro's Pinpoint, set up from Constants (pod offsets, directions, units).
        PinpointLocalizer odo = new PinpointLocalizer(hardwareMap, Constants.localizerConfig);
        odo.setPose(Pose.zero());

        tagLocalizer.start();

        telemetryM.addData("Status", "Initialized");
        telemetryM.update(telemetry);

        waitForStart();
        if (isStopRequested()) return;

        odo.setPose(Pose.zero());

        while (opModeIsActive()) {

            odo.update();

            // Heading is 0 where the robot faced at START, counter-clockwise positive.
            // MegaTag 2 really wants a FIELD heading: to get one, setPose() above with
            // the robot's real starting heading instead of Pose.zero().
            double yawDeg = Math.toDegrees(odo.pose().heading());
            limelight.updateRobotOrientation(yawDeg);

            // ---- Tag-relative offset (what you want for localizing to a tag) ----
            LimelightTagLocalizer.TagOffset offset = tagLocalizer.getOffset(DESIRED_TAG_ID);

            telemetryM.addData("Tag status", tagLocalizer.getStatus());
            if (offset != null) {
                telemetryM.addData("Tag ID", offset.tagId);
                telemetryM.addData("Forward (in)", offset.forwardIn());
                telemetryM.addData("Lateral (in)", offset.lateralIn());
                telemetryM.addData("Heading (deg)", offset.headingDeg);

                telemetryM.addData("Raw X (m)", offset.rawX);
                telemetryM.addData("Raw Y (m)", offset.rawY);
                telemetryM.addData("Raw Z (m)", offset.rawZ);
                telemetryM.addData("Raw yaw (deg)", offset.rawYaw);
                telemetryM.addData("Raw pitch (deg)", offset.rawPitch);
                telemetryM.addData("Raw roll (deg)", offset.rawRoll);
            }

            // ---- Field pose (only works if the tag ID is in the Limelight's field map) ----
            LLResult result = limelight.getLatestResult();
            if (result != null && result.isValid()) {
                telemetryM.addData("Pipeline", result.getPipelineIndex()
                        + " (" + result.getPipelineType() + ")");
                telemetryM.addData("Map tags used", result.getBotposeTagCount());

                Pose3D mt1 = result.getBotpose();
                if (mt1 != null) {
                    telemetryM.addData("MT1 X (m)", mt1.getPosition().x);
                    telemetryM.addData("MT1 Y (m)", mt1.getPosition().y);
                    telemetryM.addData("MT1 Yaw (deg)",
                            mt1.getOrientation().getYaw(AngleUnit.DEGREES));
                }

                Pose3D mt2 = result.getBotpose_MT2();
                if (mt2 != null) {
                    telemetryM.addData("MT2 X (m)", mt2.getPosition().x);
                    telemetryM.addData("MT2 Y (m)", mt2.getPosition().y);
                }
            }

            telemetryM.addData("Pinpoint yaw sent (deg)", yawDeg);
            telemetryM.update(telemetry);
        }

        tagLocalizer.stop();
    }
}

