package org.firstinspires.ftc.teamcode.Wrapper;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.List;

/**
 * Battery voltage, read every so often and smoothed with a low-pass filter.
 *
 * Reading the voltage costs a hub transaction, so it is only read once per readPeriodSec.
 * The battery also dips for a moment whenever something pulls hard (flywheel spin-up, the
 * drivetrain pushing). The filter smooths those dips out so code that scales power by the
 * voltage doesn't jump around: a dip much shorter than filterSec barely moves the value,
 * while a real, lasting drop (a tired battery) comes through after about filterSec.
 *
 * Usage:
 *   BatteryVoltage battery = new BatteryVoltage(hardwareMap);
 *   while (...) {
 *       double volts = battery.update(0.1, 0.5);   // once per loop
 *   }
 */
public class BatteryVoltage {

    private final List<VoltageSensor> sensors;
    private final ElapsedTime sinceRead = new ElapsedTime();
    private double raw = Double.NaN;
    private double filtered = Double.NaN;

    public BatteryVoltage(HardwareMap hardwareMap) {
        sensors = hardwareMap.getAll(VoltageSensor.class);
    }

    /**
     * Reads the battery if readPeriodSec has passed and updates the filter.
     * Returns the filtered voltage, or NaN if no reading has worked yet.
     */
    public double update(double readPeriodSec, double filterSec) {
        boolean first = Double.isNaN(filtered);
        if (!first && sinceRead.seconds() < readPeriodSec) return filtered;

        double dt = sinceRead.seconds();
        sinceRead.reset();
        double v = read();
        if (v <= 0) return filtered;

        raw = v;
        if (first || filterSec <= 0) {
            filtered = v;
        } else {
            // First-order low-pass: the bigger the gap between reads, the more the new one counts.
            double alpha = dt / (filterSec + dt);
            filtered += alpha * (v - filtered);
        }
        return filtered;
    }

    /** Filtered voltage from the last update(). NaN until the first good reading. */
    public double get() {
        return filtered;
    }

    /** Last unfiltered reading. NaN until the first good reading. */
    public double getRaw() {
        return raw;
    }

    /** Lowest voltage any hub reports. With an Expansion Hub, both see the same battery. */
    private double read() {
        double lowest = Double.POSITIVE_INFINITY;
        for (VoltageSensor sensor : sensors) {
            double v = sensor.getVoltage();
            if (v > 0) lowest = Math.min(lowest, v);
        }
        return Double.isInfinite(lowest) ? 0 : lowest;
    }
}
