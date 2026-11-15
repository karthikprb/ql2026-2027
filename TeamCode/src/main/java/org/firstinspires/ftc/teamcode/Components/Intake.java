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
 *   intake.handleGamepad(gamepad1);     // once per loop
 *
 * Autos and the TeleOp shot sequence drive it directly with intake(power) / setState().
 */
public class Intake {

    // ---------------- hardware config names ----------------
    public static String SERVO_NAME = "intakeServo";
    public static String MOTOR_NAME = "intakeMotor";

    // ---------------- servo positions (tune these) ----------------
    public static double DROPDOWN_LOWERED = 0.1;
    public static double DROPDOWN_RAISED = 0.3;

    // ---------------- motor powers ----------------
    public static double INTAKE_POWER = 1;         // A: full speed
    public static double INTAKE_POWER_LOW = 0.3;   // dpad left
    public static double INTAKE_POWER_MID = 0.6;   // dpad right
    public static double OUTTAKE_POWER = -1;

    public enum State { INTAKE, STOW, OUTTAKE }

    private final Servo dropdown;
    private final DcMotor motor;
    private State state = State.STOW;
    private double intakePower = INTAKE_POWER;   // power used while in INTAKE; A always puts it back to INTAKE_POWER

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
     *   A          = toggle: intake OFF -> INTAKE at full INTAKE_POWER; intake ON (any power) -> STOW
     *   Dpad left  = INTAKE at INTAKE_POWER_LOW (0.3)
     *   Dpad right = INTAKE at INTAKE_POWER_MID (0.6)
     *   B          = STOW     (dropdown up, motor off)
     *   X          = OUTTAKE  (dropdown down, motor at OUTTAKE_POWER)
     * Turning on with A is always full power, whatever dpad left / right set before.
     */
    public void handleGamepad(Gamepad gamepad) {
        if (gamepad.aWasPressed()) {
            if (state == State.INTAKE) setState(State.STOW);
            else intake(INTAKE_POWER);
        }
        if (gamepad.dpadLeftWasPressed()) intake(INTAKE_POWER_LOW);
        if (gamepad.dpadRightWasPressed()) intake(INTAKE_POWER_MID);
        if (gamepad.bWasPressed()) setState(State.STOW);
        if (gamepad.xWasPressed()) setState(State.OUTTAKE);
    }

    /** Runs the intake (dropdown down) at the given power, and remembers that power for INTAKE. */
    public void intake(double power) {
        intakePower = power;
        setState(State.INTAKE);
    }

    /** The power the intake runs at while in INTAKE (INTAKE_POWER after A, lower after dpad left / right). */
    public double getIntakePower() {
        return intakePower;
    }

    /** Puts the intake in the given state straight away. */
    public void setState(State newState) {
        state = newState;
        switch (state) {
            case INTAKE:
                dropdown.setPosition(DROPDOWN_LOWERED);
                motor.setPower(intakePower);
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
