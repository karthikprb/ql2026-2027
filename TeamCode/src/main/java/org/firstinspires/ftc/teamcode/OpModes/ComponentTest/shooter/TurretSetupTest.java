package org.firstinspires.ftc.teamcode.OpModes.ComponentTest.shooter;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.Components.Shooter;

/**
 * STEP 1 for the turret. Finds:
 *   TURRET_TICKS_PER_DEG, TURRET_DIRECTION,
 *   TURRET_FORWARD_DEG, TURRET_MIN_DEG, TURRET_MAX_DEG, TURRET_START_DEG
 *
 * Shooter's turret angles: 0 deg = full left, angles grow to the right.
 *
 * Turn the turret with the stick (plain power, no PID) and press a button at each spot:
 *   A = turret points straight FORWARD, along the robot     (zeroes the count here)
 *   B = turret points straight out the robot's RIGHT side   (exactly 90 deg from A,
 *       line it up with the robot's side rail or a square)
 *   X = turret at its full LEFT limit
 *   Y = turret at its full RIGHT limit
 * Do A first, then the rest in any order. A again starts over.
 *
 * Gamepad 1:
 *   Left stick X = turn the turret
 *   Right bumper = hold for slow
 */
@TeleOp(name = "Shooter 4: Turret Setup", group = "Shooter")
public class TurretSetupTest extends LinearOpMode {

    private static final double MAX_POWER = 0.35;
    private static final double SLOW_POWER = 0.12;
    private static final double LIMIT_MARGIN_DEG = 3;   // stay this far off each hard stop

    @Override
    public void runOpMode() {
        DcMotorEx turret = hardwareMap.get(DcMotorEx.class, Shooter.TURRET_NAME);
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addLine("Shooter 4: Turret Setup. Turret moves only with the left stick.");
        telemetry.update();
        waitForStart();

        int zero = 0;
        boolean haveForward = false;
        Integer rightSide = null, leftLimit = null, rightLimit = null;

        int lastTicks = turret.getCurrentPosition();
        String powerVsEncoder = "move the turret to check";

        while (opModeIsActive()) {
            double scale = gamepad1.right_bumper ? SLOW_POWER : MAX_POWER;
            double power = gamepad1.left_stick_x * scale;
            turret.setPower(power);

            int raw = turret.getCurrentPosition();
            int ticks = raw - zero;

            // Shooter uses one direction setting for both power and encoder, so positive
            // power has to make the count go up. Check that here.
            int change = raw - lastTicks;
            lastTicks = raw;
            if (Math.abs(power) > 0.08 && Math.abs(change) > 2) {
                powerVsEncoder = Math.signum(power) == Math.signum(change)
                        ? "OK"
                        : "BACKWARDS: + power makes the count go DOWN. Shooter can't handle this yet.";
            }

            if (gamepad1.aWasPressed()) {
                zero = raw;
                ticks = 0;
                haveForward = true;
                rightSide = leftLimit = rightLimit = null;
            }
            // Read every button each loop so an early press doesn't fire later.
            boolean b = gamepad1.bWasPressed(), x = gamepad1.xWasPressed(), y = gamepad1.yWasPressed();
            if (haveForward) {
                if (b) rightSide = ticks;
                if (x) leftLimit = ticks;
                if (y) rightLimit = ticks;
            }

            telemetry.addData("Ticks from forward", haveForward ? String.valueOf(ticks) : "press A first");
            telemetry.addData("Power vs encoder", powerVsEncoder);
            telemetry.addData("A forward", haveForward ? "set" : "-");
            telemetry.addData("B right side (90)", rightSide == null ? "-" : rightSide.toString());
            telemetry.addData("X left limit", leftLimit == null ? "-" : leftLimit.toString());
            telemetry.addData("Y right limit", rightLimit == null ? "-" : rightLimit.toString());
            telemetry.addLine();

            if (rightSide != null && Math.abs(rightSide) > 10) {
                double ticksPerDeg = Math.abs(rightSide) / 90.0;
                double direction = Math.signum(rightSide);

                telemetry.addLine("Paste into Shooter.java:");
                telemetry.addLine(String.format("TURRET_TICKS_PER_DEG = %.4f;", ticksPerDeg));
                telemetry.addLine(String.format("TURRET_DIRECTION = %.1f;", direction));

                if (leftLimit != null && rightLimit != null) {
                    // Degrees left / right of forward, both as positive numbers.
                    double leftDeg = -direction * leftLimit / ticksPerDeg;
                    double rightDeg = direction * rightLimit / ticksPerDeg;
                    if (leftDeg <= 0 || rightDeg <= 0) {
                        telemetry.addLine("Limits look swapped: X should be LEFT of forward, Y RIGHT.");
                    } else {
                        // 0 deg is the left hard stop, so forward is leftDeg.
                        double forward = leftDeg;
                        telemetry.addLine(String.format("TURRET_FORWARD_DEG = %.1f;", forward));
                        telemetry.addLine(String.format("TURRET_MIN_DEG = %.1f;", LIMIT_MARGIN_DEG));
                        telemetry.addLine(String.format("TURRET_MAX_DEG = %.1f;", leftDeg + rightDeg - LIMIT_MARGIN_DEG));
                        telemetry.addLine(String.format("TURRET_START_DEG = %.1f;", forward));
                        telemetry.addLine("  (START = FORWARD means: always point the turret");
                        telemetry.addLine("   forward before pressing INIT. The count resets there.)");
                        telemetry.addLine();
                        telemetry.addData("Turret now (Shooter deg)", "%.1f",
                                forward + direction * ticks / ticksPerDeg);
                    }
                } else {
                    telemetry.addLine("Press X and Y at the limits to get the angle range.");
                }
            } else if (haveForward) {
                telemetry.addLine("Turn exactly 90 deg right and press B.");
            }
            telemetry.update();
        }
        turret.setPower(0);
    }
}
