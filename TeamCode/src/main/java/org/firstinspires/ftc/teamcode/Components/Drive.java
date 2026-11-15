package org.firstinspires.ftc.teamcode.Components;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.follower.drivetrain.DrivePowers;
import org.firstinspires.ftc.teamcode.follower.follower.Follower;
import org.firstinspires.ftc.teamcode.follower.follower.ManualDrive;

/**
 * Driver-controlled driving on top of the Pedro follower. All the driving gamepad buttons
 * live here, not in the OpMode.
 *
 * Usage in an OpMode:
 *   Follower follower = Constants.create(hardwareMap);
 *   Drive drive = new Drive(follower);
 *   ...
 *   follower.update();
 *   drive.handleGamepad(gamepad1);     // once per loop
 *
 * Gamepad:
 *   Left stick    = drive / strafe
 *   Right stick X = turn
 *   Right trigger = hold for slow mode (SLOW_MODE_SCALE)
 *   Options       = toggle field-centric driving (forward = heading 0 of the follower's pose)
 *
 * @Configurable: SLOW_MODE_SCALE shows up under "Drive" in Panels.
 */
@Configurable
public class Drive {

    /** Drive power multiplier while the right trigger is held. */
    public static double SLOW_MODE_SCALE = 0.4;

    private final Follower follower;
    private boolean fieldCentric = false;
    private boolean slow = false;

    public Drive(Follower follower) {
        this.follower = follower;
    }

    /** Reads the sticks and sends drive powers to the follower. Call once per loop, after follower.update(). */
    public void handleGamepad(Gamepad gamepad) {
        if (gamepad.optionsWasPressed()) fieldCentric = !fieldCentric;
        slow = gamepad.right_trigger > 0.5;

        double scale = slow ? SLOW_MODE_SCALE : 1.0;
        DrivePowers powers = new DrivePowers(
                -gamepad.left_stick_y * scale,    // forward
                -gamepad.left_stick_x * scale,    // strafe (left = positive)
                -gamepad.right_stick_x * scale);  // turn (left = positive)
        if (fieldCentric) powers = ManualDrive.fieldCentric(powers, follower.pose().heading());
        follower.manual(powers);
    }

    public boolean isFieldCentric() {
        return fieldCentric;
    }

    public boolean isSlow() {
        return slow;
    }

    /** Stops the drive motors. Call when the OpMode ends. */
    public void stop() {
        follower.stop();
        follower.update();
    }
}
