package org.firstinspires.ftc.teamcode.OpModes.ComponentTest.shooter;

import com.bylazar.gamepad.GamepadManager;
import com.qualcomm.robotcore.hardware.Gamepad;

/**
 * One gamepad that works from BOTH the Driver Station and a controller plugged into the
 * computer running Panels. Whichever one you press counts. Not an OpMode.
 *
 * Usage:
 *   PanelsPad pad1 = new PanelsPad(gamepad1, PanelsGamepad.INSTANCE.getFirstManager());
 *   while (opModeIsActive()) {
 *       pad1.update();                       // once at the top of every loop
 *       if (pad1.aWasPressed()) ...          // true for one loop per press
 *       double x = pad1.raw().left_stick_x;  // sticks, triggers, held buttons
 *   }
 *
 * The "was pressed" checks are done here instead of with the SDK's own aWasPressed(),
 * because the combined gamepad is a fresh copy every loop and has no press history.
 */
public class PanelsPad {

    private final Gamepad driverStation;
    private final GamepadManager panels;

    private Gamepad now = new Gamepad();
    private boolean a, b, x, y, up, down, left, right, lb, rb;   // last loop's buttons

    public PanelsPad(Gamepad driverStation, GamepadManager panels) {
        this.driverStation = driverStation;
        this.panels = panels;
    }

    /** Reads both gamepads. Call once at the top of every loop, before any checks. */
    public void update() {
        a = now.a; b = now.b; x = now.x; y = now.y;
        up = now.dpad_up; down = now.dpad_down; left = now.dpad_left; right = now.dpad_right;
        lb = now.left_bumper; rb = now.right_bumper;
        now = panels.asCombinedFTCGamepad(driverStation);
    }

    /** The combined gamepad this loop, for sticks, triggers and held buttons. */
    public Gamepad raw() {
        return now;
    }

    public boolean aWasPressed() { return now.a && !a; }
    public boolean bWasPressed() { return now.b && !b; }
    public boolean xWasPressed() { return now.x && !x; }
    public boolean yWasPressed() { return now.y && !y; }
    public boolean dpadUpWasPressed() { return now.dpad_up && !up; }
    public boolean dpadDownWasPressed() { return now.dpad_down && !down; }
    public boolean dpadLeftWasPressed() { return now.dpad_left && !left; }
    public boolean dpadRightWasPressed() { return now.dpad_right && !right; }
    public boolean leftBumperWasPressed() { return now.left_bumper && !lb; }
    public boolean rightBumperWasPressed() { return now.right_bumper && !rb; }
}
