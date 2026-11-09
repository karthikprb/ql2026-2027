package org.firstinspires.ftc.teamcode.OpModes.ComponentTest.shooter;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * A short list of numbers (PID gains etc.) you can change live from a gamepad.
 * Used by the shooter PID tests. Not an OpMode.
 *
 *   Dpad left / right = pick which value to change
 *   Right bumper      = value x 1.25   (from 0 it jumps to a sensible starting value)
 *   Left bumper       = value / 1.25
 *   Y                 = set the picked value to 0
 */
class TunableValues {

    private static final double STEP = 1.25;

    private final String[] names;
    private final double[] values;
    private final double[] seeds;
    private int selected = 0;

    /**
     * @param names  what each value is called in telemetry
     * @param values starting values
     * @param seeds  what a value jumps to when you raise it from 0
     */
    TunableValues(String[] names, double[] values, double[] seeds) {
        this.names = names;
        this.values = values.clone();
        this.seeds = seeds;
    }

    /** Reads the buttons listed above. Call once per loop. */
    void handle(Gamepad g) {
        int n = values.length;
        if (g.dpadRightWasPressed()) selected = (selected + 1) % n;
        if (g.dpadLeftWasPressed()) selected = (selected + n - 1) % n;
        if (g.rightBumperWasPressed()) {
            values[selected] = values[selected] == 0 ? seeds[selected] : values[selected] * STEP;
        }
        if (g.leftBumperWasPressed()) values[selected] /= STEP;
        if (g.yWasPressed()) values[selected] = 0;
    }

    double get(int index) {
        return values[index];
    }

    void addTelemetry(Telemetry t) {
        for (int i = 0; i < values.length; i++) {
            t.addData((i == selected ? ">> " : "   ") + names[i], fmt(values[i]));
        }
    }

    static String fmt(double v) {
        return String.format("%.6g", v);
    }
}
