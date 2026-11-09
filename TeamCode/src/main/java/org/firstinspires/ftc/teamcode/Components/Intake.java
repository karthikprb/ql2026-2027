package org.firstinspires.ftc.teamcode.Components;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Intake with a servo dropdown and a motor that spins the intake.
 *
 * Three states, each picked directly by a gamepad button. There is no order
 * to them: whichever button is pressed last is the state the intake is in.
 *
 *   INTAKE  - dropdown lowered, motor pulls game pieces in
 *   STOW    - dropdown raised, motor off
 *   OUTTAKE - dropdown lowered, motor spins backwards to spit pieces out
 *
 * Usage in an OpMode:
 *   Intake intake = new Intake(hardwareMap);
 *   ...
 *   intake.update(gamepad2);     // once per loop
 */
public class Intake {

    // ---------------- hardware config names ----------------
    public static String SERVO_NAME = "intakeServo";
    public static String MOTOR_NAME = "intakeMotor";

    // ---------------- servo positions (tune these) ----------------
    public static double DROPDOWN_LOWERED = 0.0;
    public static double DROPDOWN_RAISED = 0.5;

    // ---------------- motor powers ----------------
    public static double INTAKE_POWER = 1.0;
    public static double OUTTAKE_POWER = -1.0;

    public enum State { INTAKE, STOW, OUTTAKE }

    private final Servo dropdown;
    private final DcMotor motor;
    private State state = State.STOW;

    public Intake(HardwareMap hardwareMap) {
        dropdown = hardwareMap.get(Servo.class, SERVO_NAME);
        motor = hardwareMap.get(DcMotor.class, MOTOR_NAME);
        motor.setDirection(DcMotorSimple.Direction.FORWARD);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        setState(State.STOW);
    }

    /**
     * Reads the gamepad and applies the matching state. Call once per loop.
     *
     *   Right bumper = INTAKE
     *   Left bumper  = STOW
     *   B / circle   = OUTTAKE
     */
    public void update(Gamepad gamepad) {
        if (gamepad.right_bumper) {
            setState(State.INTAKE);
        } else if (gamepad.left_bumper) {
            setState(State.STOW);
        } else if (gamepad.b) {
            setState(State.OUTTAKE);
        }
    }

    /** Puts the intake in the given state straight away. */
    public void setState(State newState) {
        state = newState;
        switch (state) {
            case INTAKE:
                dropdown.setPosition(DROPDOWN_LOWERED);
                motor.setPower(INTAKE_POWER);
                break;
            case STOW:
                dropdown.setPosition(DROPDOWN_RAISED);
                motor.setPower(0);
                break;
            case OUTTAKE:
                dropdown.setPosition(DROPDOWN_LOWERED);
                motor.setPower(OUTTAKE_POWER);
                break;
        }
    }

    public State getState() {
        return state;
    }

    /** Raises the dropdown and stops the motor. Call when the OpMode ends. */
    public void stop() {
        setState(State.STOW);
    }
}
