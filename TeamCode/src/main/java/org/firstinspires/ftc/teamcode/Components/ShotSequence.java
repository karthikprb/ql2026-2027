package org.firstinspires.ftc.teamcode.Components;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * The one-button TeleOp shot, on gamepad 1 Y:
 *   1. HOOD_UP:  hood goes up. After HOOD_UP_TIME (so the servo has moved) ...
 *   2. SPIN_UP:  flywheels turn on at Shooter.SHOOTER_RPM. Once they are at speed
 *                (or SPINUP_TIMEOUT passes, so a low battery can't stall it) ...
 *   3. FEED:     intake runs at full power, feeding the balls in. After FEED_TIME ...
 *   4. done:     hood comes back down, intake stops, flywheels go back to how they were
 *                before Y was pressed (still on if the driver had them on with RB, else off).
 *
 * Y again while it is running cancels it (same clean-up as finishing). Turning the shooter
 * off with LB during the sequence cancels it too.
 *
 * Usage in an OpMode, once per loop, after shooter.handleGamepad() and intake.handleGamepad():
 *   shot.handleGamepad(gamepad1);
 * It drives Shooter and Intake through their setters, so shooter.update() still runs the PIDs.
 *
 * @Configurable: the times below show up under "ShotSequence" in Panels.
 */
@Configurable
public class ShotSequence {

    public static double HOOD_UP_TIME   = 0.3;   // s, let the hood servo get up before the flywheels start
    public static double SPINUP_TIMEOUT = 3.0;   // s, feed anyway if the flywheels never reach SHOOTER_RPM
    public static double FEED_TIME      = 3.0;   // s the intake feeds before the hood comes back down

    public enum State { IDLE, HOOD_UP, SPIN_UP, FEED }

    private final Shooter shooter;
    private final Intake intake;
    private State state = State.IDLE;
    private final ElapsedTime timer = new ElapsedTime();
    private boolean shooterWasOn = false;   // flywheel state to go back to when the shot is done

    public ShotSequence(Shooter shooter, Intake intake) {
        this.shooter = shooter;
        this.intake = intake;
    }

    /** Y starts the shot, or cancels it if one is running. Also moves the sequence along. Call once per loop. */
    public void handleGamepad(Gamepad gamepad) {
        if (gamepad.yWasPressed()) {
            if (state == State.IDLE) start();
            else cancel();
        }
        update();
    }

    /** Begin the shot: hood up first. */
    public void start() {
        shooterWasOn = shooter.isShooterOn();
        shooter.setHoodUp(true);
        timer.reset();
        state = State.HOOD_UP;
    }

    /** Stop the shot now: hood down, intake off, flywheels back to how they were. */
    public void cancel() {
        finish();
    }

    /** Moves the sequence along one step. handleGamepad() calls this; call it yourself if you start() from code. */
    public void update() {
        switch (state) {
            case HOOD_UP:
                if (timer.seconds() >= HOOD_UP_TIME) {
                    shooter.setShooterOn(true);
                    timer.reset();
                    state = State.SPIN_UP;
                }
                break;
            case SPIN_UP:
                if (!shooter.isShooterOn()) { shooterWasOn = false; finish(); break; }   // LB during the shot
                if (shooter.flywheelAtSpeed() || timer.seconds() >= SPINUP_TIMEOUT) {
                    intake.intake(Intake.INTAKE_POWER);
                    timer.reset();
                    state = State.FEED;
                }
                break;
            case FEED:
                if (!shooter.isShooterOn()) { shooterWasOn = false; finish(); break; }
                if (timer.seconds() >= FEED_TIME) finish();
                break;
            default:
                break;
        }
    }

    private void finish() {
        shooter.setHoodUp(false);
        intake.setState(Intake.State.STOW);
        shooter.setShooterOn(shooterWasOn);
        state = State.IDLE;
    }

    public State getState() {
        return state;
    }

    public boolean isRunning() {
        return state != State.IDLE;
    }

    /** Seconds spent in the current step, for telemetry. */
    public double secondsInState() {
        return state == State.IDLE ? 0 : timer.seconds();
    }
}
