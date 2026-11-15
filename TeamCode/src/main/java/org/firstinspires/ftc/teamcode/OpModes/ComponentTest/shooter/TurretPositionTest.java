package org.firstinspires.ftc.teamcode.OpModes.ComponentTest.shooter;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Components.Shooter;

/**
 * Finds the turret's two ends: TURRET_MIN_TICKS (counterclockwise end) and TURRET_MAX_TICKS
 * (clockwise end). Uses the SDK's RUN_TO_POSITION to jog, so it works before the turret PID
 * is tuned (Shooter itself uses its own PID: tune it next with "Shooter 5: Turret PID").
 *
 * How to use:
 *   1. Point the turret straight forward (TURRET_START_DEG, 90 deg) and press INIT.
 *      The encoder count resets to 0 there.
 *   2. Jog it COUNTERCLOCKWISE (left, seen from above) to the end you want. Stop before
 *      it hits anything. Press A to save the ticks. Measure the angle there: that's
 *      TURRET_MIN_DEG (e.g. 60 if it turned 30 deg left of forward).
 *   3. Jog it CLOCKWISE (right) to the other end. Press B to save the ticks. Measure the
 *      angle: that's TURRET_MAX_DEG (e.g. 120 if it turned 30 deg right of forward).
 *   4. Copy the lines from telemetry into Shooter.java, and type in the two angles you measured.
 *   5. Press Y to go back to 0 and check it comes home to forward.
 *
 * Gamepad 1:
 *   Dpad left / right = jog -/+ FINE_STEP ticks
 *   Bumpers           = jog -/+ COARSE_STEP ticks (left bumper = minus)
 *   A                 = save current ticks as the COUNTERCLOCKWISE end (TURRET_MIN_TICKS)
 *   B                 = save current ticks as the CLOCKWISE end        (TURRET_MAX_TICKS)
 *   Y                 = go back to 0 ticks (start / forward)
 *
 * Plus or minus ticks doesn't matter: whichever way the motor counts, Shooter works it out
 * from the signs of the two saved ends. Start with a small POWER so you don't hit a hard stop fast.
 */
@Configurable
@TeleOp(name = "Shooter 4: Turret Position", group = "Shooter")
public class TurretPositionTest extends LinearOpMode {

    public static double POWER = 0.3;
    public static int FINE_STEP = 2;
    public static int COARSE_STEP = 10;

    @Override
    public void runOpMode() {
        telemetry = new JoinedTelemetry(PanelsTelemetry.INSTANCE.getFtcTelemetry(), telemetry);

        DcMotorEx turret = hardwareMap.get(DcMotorEx.class, Shooter.TURRET_NAME);
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setTargetPosition(0);
        turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addLine("Turret should be pointing FORWARD right now. Count resets to 0 here.");
        telemetry.addLine("After start: dpad / bumpers jog, A = save CCW end, B = save CW end, Y = home.");
        telemetry.update();
        waitForStart();

        int target = 0;
        Integer ccwTicks = null;   // saved with A
        Integer cwTicks = null;    // saved with B

        while (opModeIsActive()) {
            if (gamepad1.dpadRightWasPressed()) target += FINE_STEP;
            if (gamepad1.dpadLeftWasPressed()) target -= FINE_STEP;
            if (gamepad1.rightBumperWasPressed()) target += COARSE_STEP;
            if (gamepad1.leftBumperWasPressed()) target -= COARSE_STEP;
            if (gamepad1.yWasPressed()) target = 0;

            turret.setTargetPosition(target);
            turret.setPower(Range.clip(POWER, 0, 1));

            int ticks = turret.getCurrentPosition();
            if (gamepad1.aWasPressed()) ccwTicks = ticks;
            if (gamepad1.bWasPressed()) cwTicks = ticks;

            telemetry.addData("Target ticks", target);
            telemetry.addData("Current ticks", ticks);
            telemetry.addData("Moving", turret.isBusy() ? "yes" : "no");
            telemetry.addData("Angle with current Shooter values", "%.1f deg",
                    Shooter.turretDegreesForTicks(ticks));
            telemetry.addLine();

            telemetry.addData("CCW end (A)", ccwTicks == null ? "-" : ccwTicks + " ticks");
            telemetry.addData("CW end (B)", cwTicks == null ? "-" : cwTicks + " ticks");
            if (ccwTicks != null || cwTicks != null) {
                telemetry.addLine();
                telemetry.addLine("Paste into Shooter.java:");
                if (ccwTicks != null) {
                    telemetry.addLine(String.format("TURRET_MIN_TICKS = %d;", ccwTicks));
                    telemetry.addLine("TURRET_MIN_DEG = <90 minus how far left it turned>;");
                }
                if (cwTicks != null) {
                    telemetry.addLine(String.format("TURRET_MAX_TICKS = %d;", cwTicks));
                    telemetry.addLine("TURRET_MAX_DEG = <90 plus how far right it turned>;");
                }
            }
            if (ccwTicks != null && cwTicks != null
                    && Math.signum(ccwTicks) == Math.signum(cwTicks)) {
                telemetry.addLine();
                telemetry.addLine("WARNING: both ends have the same sign. The turret should cross 0");
                telemetry.addLine("(forward) between them. Re-INIT with it pointing forward.");
            }

            // Plain numbers for the Panels graph.
            telemetry.addData("target", target);
            telemetry.addData("ticks", ticks);
            telemetry.update();
        }
        turret.setPower(0);
    }
}
