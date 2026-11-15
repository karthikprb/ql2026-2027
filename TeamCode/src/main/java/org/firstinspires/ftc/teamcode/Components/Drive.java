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
 * Two headings are given to the constructor, both in radians on the field:
 *   homeHeading   = the robot's normal shooting heading (270 blue, 90 red). Dpad down resets
 *                   the odometry heading to it (x and y are kept): square the robot up facing
 *                   its cell, then press dpad down.
 *   fieldForward  = the field direction that "stick forward" drives in while field-centric
 *                   (180 for blue = toward the x = 0 wall, 0 for red), whichever way the robot
 *                   itself is facing. Driving starts FIELD-centric; Options switches.
 *
 * Usage in an OpMode:
 *   Follower follower = Constants.create(hardwareMap);
 *   Drive drive = new Drive(follower, homeHeadingRad, fieldForwardRad);
 *   ...
 *   follower.update();
 *   drive.handleGamepad(gamepad1);     // once per loop
 *
 * Gamepad:
 *   Left stick    = drive / strafe
 *   Right stick X = turn
 *   Right trigger = hold for slow mode (SLOW_MODE_SCALE)
 *   Options       = toggle field-centric (starts ON) / robot-centric
 *   Dpad down     = reset heading to homeHeading, no matter what it reads now
 *
 * @Configurable: SLOW_MODE_SCALE shows up under "Drive" in Panels.
 */
@Configurable
public class Drive {

    /** Drive power multiplier while the right trigger is held. */
    public static double SLOW_MODE_SCALE = 0.4;

    private final Follower follower;
    private final double homeHeading;    // radians
    private final double fieldForward;   // radians
    private boolean fieldCentric = true;   // starts field-centric
    private boolean slow = false;
    private int headingResets = 0;

    /** homeHeading 270 deg, field-centric forward 180 deg (blue defaults). */
    public Drive(Follower follower) {
        this(follower, Math.toRadians(270), Math.toRadians(180));
    }

    public Drive(Follower follower, double homeHeadingRad, double fieldForwardRad) {
        this.follower = follower;
        this.homeHeading = homeHeadingRad;
        this.fieldForward = fieldForwardRad;
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
        // Field-centric: stick forward = fieldForward on the field, whichever way the robot faces.
        if (fieldCentric) powers = ManualDrive.fieldCentric(powers, follower.pose().heading(), -fieldForward);
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

    public double getFieldForward() {
        return fieldForward;
    }

    public boolean isFieldCentric() {
        return fieldCentric;
    }

    public void setFieldCentric(boolean on) {
        fieldCentric = on;
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
