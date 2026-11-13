package org.firstinspires.ftc.teamcode.OpModes.ComponentTest;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.gamepad.PanelsGamepad;
import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.OpModes.ComponentTest.shooter.PanelsPad;

/**
 * Drives up to three servos at once. Everything is changed live from Panels.
 *
 * In Panels > Configurables > "ServoTester":
 *   SERVO_1_NAME / SERVO_2_NAME / SERVO_3_NAME  = hardware map names. Change one and the
 *                                                 tester re-opens that servo on the next loop.
 *   SERVO_1_POS  / SERVO_2_POS  / SERVO_3_POS   = 0..1 position. Applied as soon as it changes.
 *   SERVO_1_REVERSED / ...                      = flip the servo's direction.
 *   SERVO_1_ENABLED  / ...                      = turn a slot off if nothing is plugged in there.
 *
 * A servo does NOT move until its position is first changed (from Panels or the gamepad),
 * or until you press A to send every enabled servo its current position. That way a servo
 * doesn't jump the moment you press start.
 *
 * Gamepad 1 (Driver Station or a controller plugged into the Panels computer):
 *   Dpad left / right  = pick servo 1, 2 or 3
 *   Dpad up / down     = selected servo position +/- FINE_STEP
 *   Bumpers            = selected servo position +/- COARSE_STEP (right = up)
 *   A                  = send every enabled servo its current position
 *   X                  = selected servo to 0.0
 *   Y                  = selected servo to 1.0
 *   B                  = selected servo to 0.5
 *
 * Gamepad edits write back into the SERVO_n_POS fields, so Panels always shows the real
 * position. Panels changes are NOT saved: copy the numbers you like into your own code.
 */
@Configurable
@TeleOp(name = "Servo Tester", group = "Test")
public class ServoTester extends LinearOpMode {

    // ---- Servo 1 ----
    public static String SERVO_1_NAME = "servo1";
    public static double SERVO_1_POS = 0.5;
    public static boolean SERVO_1_REVERSED = false;
    public static boolean SERVO_1_ENABLED = true;

    // ---- Servo 2 ----
    public static String SERVO_2_NAME = "servo2";
    public static double SERVO_2_POS = 0.5;
    public static boolean SERVO_2_REVERSED = false;
    public static boolean SERVO_2_ENABLED = true;

    // ---- Servo 3 ----
    public static String SERVO_3_NAME = "servo3";
    public static double SERVO_3_POS = 0.5;
    public static boolean SERVO_3_REVERSED = false;
    public static boolean SERVO_3_ENABLED = true;

    // ---- Gamepad step sizes ----
    public static double FINE_STEP = 0.01;
    public static double COARSE_STEP = 0.05;

    /** One servo slot: tracks which name is open and what was last sent. */
    private class Slot {
        final int index;            // 1..3, for telemetry
        Servo servo = null;
        String openedName = null;   // the name `servo` was opened with
        String error = null;        // why the name couldn't be opened, or null
        Double lastSent = null;     // last position written to the servo, null = never
        Boolean lastReversed = null;
        boolean armed = false;      // true once the user has changed the position or pressed A

        Slot(int index) {
            this.index = index;
        }

        /** Re-opens the servo if the configured name changed. */
        void refresh(String name, boolean enabled) {
            if (!enabled) {
                servo = null;
                openedName = null;
                error = null;
                return;
            }
            if (name == null) name = "";
            name = name.trim();
            if (name.equals(openedName)) return;

            openedName = name;
            lastSent = null;
            lastReversed = null;
            try {
                servo = hardwareMap.get(Servo.class, name);
                error = null;
            } catch (Exception e) {
                servo = null;
                error = "not in hardware map";
            }
        }

        /** Writes direction and position, but only when something changed and we're armed. */
        void apply(double pos, boolean reversed) {
            if (servo == null || !armed) return;
            if (lastReversed == null || lastReversed != reversed) {
                servo.setDirection(reversed ? Servo.Direction.REVERSE : Servo.Direction.FORWARD);
                lastReversed = reversed;
            }
            if (lastSent == null || lastSent != pos) {
                servo.setPosition(pos);
                lastSent = pos;
            }
        }

        String status(String name, double pos, boolean enabled) {
            if (!enabled) return "disabled";
            if (servo == null) return "\"" + name + "\": " + error;
            String sent = lastSent == null ? "not sent yet" : String.format("sent %.3f", lastSent);
            return String.format("\"%s\"  target %.3f  (%s)", name, pos, sent);
        }
    }

    @Override
    public void runOpMode() {
        // Every telemetry line goes to both the Driver Station and Panels.
        telemetry = new JoinedTelemetry(PanelsTelemetry.INSTANCE.getFtcTelemetry(), telemetry);
        // Gamepad works from the Driver Station or a controller plugged into the Panels computer.
        PanelsPad pad1 = new PanelsPad(gamepad1, PanelsGamepad.INSTANCE.getFirstManager());

        Slot[] slots = { new Slot(1), new Slot(2), new Slot(3) };
        int selected = 0;   // 0..2

        // Open whatever names are configured right now so init can report missing ones.
        for (int i = 0; i < 3; i++) slots[i].refresh(name(i), enabled(i));

        telemetry.addLine("Servo Tester. Servos won't move until a position changes or you press A.");
        for (int i = 0; i < 3; i++) {
            telemetry.addData("Servo " + (i + 1), slots[i].status(name(i), pos(i), enabled(i)));
        }
        telemetry.addLine("Edit names / positions in Panels > Configurables > ServoTester.");
        telemetry.update();
        waitForStart();

        // Remember the Panels positions from last loop so we can tell when Panels changed one.
        double[] lastPos = { pos(0), pos(1), pos(2) };

        while (opModeIsActive()) {
            pad1.update();

            // ---- Gamepad: pick a servo and nudge it ----
            if (pad1.dpadRightWasPressed()) selected = (selected + 1) % 3;
            if (pad1.dpadLeftWasPressed()) selected = (selected + 2) % 3;

            double p = pos(selected);
            boolean nudged = false;
            if (pad1.dpadUpWasPressed()) { p += FINE_STEP; nudged = true; }
            if (pad1.dpadDownWasPressed()) { p -= FINE_STEP; nudged = true; }
            if (pad1.rightBumperWasPressed()) { p += COARSE_STEP; nudged = true; }
            if (pad1.leftBumperWasPressed()) { p -= COARSE_STEP; nudged = true; }
            if (pad1.xWasPressed()) { p = 0.0; nudged = true; }
            if (pad1.yWasPressed()) { p = 1.0; nudged = true; }
            if (pad1.bWasPressed()) { p = 0.5; nudged = true; }
            if (nudged) {
                setPos(selected, Range.clip(p, 0, 1));
                slots[selected].armed = true;
            }

            if (pad1.aWasPressed()) {
                for (Slot s : slots) s.armed = true;
            }

            // ---- Re-read everything from the static fields (Panels may have changed them) ----
            for (int i = 0; i < 3; i++) {
                double target = Range.clip(pos(i), 0, 1);
                if (target != pos(i)) setPos(i, target);   // keep Panels showing a legal value

                if (target != lastPos[i]) {
                    slots[i].armed = true;              // Panels moved it: that counts as intent
                    lastPos[i] = target;
                }

                slots[i].refresh(name(i), enabled(i));
                slots[i].apply(target, reversed(i));
            }

            // ---- Telemetry ----
            for (int i = 0; i < 3; i++) {
                String marker = (i == selected) ? "> " : "  ";
                telemetry.addData(marker + "Servo " + (i + 1), slots[i].status(name(i), pos(i), enabled(i)));
            }
            telemetry.addLine();
            telemetry.addData("Selected (dpad L/R)", "Servo " + (selected + 1));
            telemetry.addLine("Dpad U/D = fine, bumpers = coarse, X=0 B=0.5 Y=1, A = send all.");
            telemetry.update();
        }
    }

    // Small accessors so the loop can index the three slots' static fields.
    private static String name(int i) {
        switch (i) {
            case 0: return SERVO_1_NAME;
            case 1: return SERVO_2_NAME;
            default: return SERVO_3_NAME;
        }
    }

    private static double pos(int i) {
        switch (i) {
            case 0: return SERVO_1_POS;
            case 1: return SERVO_2_POS;
            default: return SERVO_3_POS;
        }
    }

    private static void setPos(int i, double v) {
        switch (i) {
            case 0: SERVO_1_POS = v; break;
            case 1: SERVO_2_POS = v; break;
            default: SERVO_3_POS = v; break;
        }
    }

    private static boolean reversed(int i) {
        switch (i) {
            case 0: return SERVO_1_REVERSED;
            case 1: return SERVO_2_REVERSED;
            default: return SERVO_3_REVERSED;
        }
    }

    private static boolean enabled(int i) {
        switch (i) {
            case 0: return SERVO_1_ENABLED;
            case 1: return SERVO_2_ENABLED;
            default: return SERVO_3_ENABLED;
        }
    }
}
