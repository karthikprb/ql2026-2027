package org.firstinspires.ftc.teamcode.Components;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.follower.drivetrain.DrivePowers;
import org.firstinspires.ftc.teamcode.follower.follower.Follower;
import org.firstinspires.ftc.teamcode.follower.follower.ManualDrive;
import org.firstinspires.ftc.teamcode.follower.math.Pose;

/**
 * Driver-controlled driving on top of the Pedro follower. All the driving gamepad buttons
 * live here, not in the OpMode.
 *
 * homeHeading is the robot's normal shooting heading on the field (270 deg = facing the
 * audience, straight at the far HIVE cell). It is used two ways:
 *   - Field-centric driving: stick forward = that direction, whichever way the robot faces.
 *   - Dpad down: resets the odometry heading to it (x and y are kept). Use it whenever the
 *     heading has drifted: square the robot up facing the far cell, then press dpad down.
 *
 * Usage in an OpMode:
 *   Follower follower = Constants.create(hardwareMap);
 *   Drive drive = new Drive(follower, startPose.heading());
 *   ...
 *   follower.update();
 *   drive.handleGamepad(gamepad1);     // once per loop
 *
 * Gamepad:
 *   Left stick    = drive / strafe
 *   Right stick X = turn
 *   Right trigger = hold for slow mode (SLOW_MODE_SCALE)
 *   Options       = toggle field-centric driving (forward = homeHeading)
 *   Dpad down     = reset heading to homeHeading, no matter what it reads now
 *
 * @Configurable: SLOW_MODE_SCALE shows up under "Drive" in Panels.
 */
@Configurable
public class Drive {

    /** Drive power multiplier while the right trigger is held. */
    public static double SLOW_MODE_SCALE = 0.4;

    private final Follower follower;
    private final double homeHeading;   // radians
    private boolean fieldCentric = false;
    private boolean slow = false;
    private int headingResets = 0;

    /** homeHeading defaults to 270 deg (facing the audience). */
    public Drive(Follower follower) {
        this(follower, Math.toRadians(270));
    }

    public Drive(Follower follower, double homeHeadingRad) {
        this.follower = follower;
        this.homeHeading = homeHeadingRad;
    }

    /** Reads the sticks and buttons and sends drive powers to the follower. Call once per loop, after follower.update(). */
    public void handleGamepad(Gamepad gamepad) {
        if (gamepad.optionsWasPressed()) fieldCentric = !fieldCentric;
        if (gamepad.dpadDownWasPressed()) resetHeading();
        slow = gamepad.right_trigger > 0.5;

        double scale = slow ? SLOW_MODE_SCALE : 1.0;
        DrivePowers powers = new DrivePowers(
                -gamepad.left_stick_y * scale,    // forward
                -gamepad.left_stick_x * scale,    // strafe (left = positive)
                -gamepad.right_stick_x * scale);  // turn (left = positive)
        // Field-centric: stick forward = homeHeading on the field, whichever way the robot faces.
        if (fieldCentric) powers = ManualDrive.fieldCentric(powers, follower.pose().heading(), -homeHeading);
        follower.manual(powers);
    }

    /** Keeps the robot's x / y and sets the odometry heading to homeHeading. */
    public void resetHeading() {
        Pose now = follower.pose();
        follower.setPose(new Pose(now.x(), now.y(), homeHeading));
        headingResets++;
    }

    public double getHomeHeading() {
        return homeHeading;
    }

    public boolean isFieldCentric() {
        return fieldCentric;
    }

    public boolean isSlow() {
        return slow;
    }

    /** How many times dpad down has reset the heading this run (for telemetry). */
    public int getHeadingResets() {
        return headingResets;
    }

    /** Stops the drive motors. Call when the OpMode ends. */
    public void stop() {
        follower.stop();
        follower.update();
    }
}
