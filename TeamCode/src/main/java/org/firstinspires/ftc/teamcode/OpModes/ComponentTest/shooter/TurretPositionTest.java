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
 * Sends the turret motor to a tick count set in Panels, so you can measure how far it turns
 * per tick. Same RUN_TO_POSITION control as Shooter, no PID of our own.
 *
 * Panels > Configurables > TurretPositionTest:
 *   TARGET_TICKS  = encoder count to go to (0 = where the turret was at init)
 *   POWER         = power for RUN_TO_POSITION, 0..1
 *   MEASURED_DEG  = type in how many degrees the turret actually turned, and the telemetry
 *                   works out TURRET_TICKS_PER_DEG and TURRET_DIRECTION for you
 *
 * How to use:
 *   1. Point the turret straight forward and press INIT. The count resets to 0 there.
 *   2. Set TARGET_TICKS to something like 200 and watch it move.
 *   3. Measure the angle it turned. Positive = turned RIGHT (toward 180 deg in Shooter).
 *      Type it into MEASURED_DEG. Copy the two numbers from telemetry into Shooter.java.
 *   4. Set TARGET_TICKS back to 0 and check it comes home.
 *
 * Start with a small POWER (0.3) and small ticks so you don't hit a hard stop fast.
 */
@Configurable
@TeleOp(name = "Shooter 4: Turret Position", group = "Shooter")
public class TurretPositionTest extends LinearOpMode {

    public static int TARGET_TICKS = 0;
    public static double POWER = 0.3;
    public static double MEASURED_DEG = 0;

    @Override
    public void runOpMode() {
        telemetry = new JoinedTelemetry(PanelsTelemetry.INSTANCE.getFtcTelemetry(), telemetry);

        DcMotorEx turret = hardwareMap.get(DcMotorEx.class, Shooter.TURRET_NAME);
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setTargetPosition(0);
        turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addLine("Turret should be pointing FORWARD right now. Count resets to 0 here.");
        telemetry.addLine("Set TARGET_TICKS in Panels > TurretPositionTest after start.");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            turret.setTargetPosition(TARGET_TICKS);
            turret.setPower(Range.clip(POWER, 0, 1));

            int ticks = turret.getCurrentPosition();

            telemetry.addData("Target ticks", TARGET_TICKS);
            telemetry.addData("Current ticks", ticks);
            telemetry.addData("Error (ticks)", TARGET_TICKS - ticks);
            telemetry.addData("Moving", turret.isBusy() ? "yes" : "no");
            telemetry.addLine();

            if (MEASURED_DEG != 0 && ticks != 0) {
                double ticksPerDeg = Math.abs(ticks / MEASURED_DEG);
                double direction = Math.signum(ticks) == Math.signum(MEASURED_DEG) ? 1.0 : -1.0;
                telemetry.addLine("Paste into Shooter.java:");
                telemetry.addLine(String.format("TURRET_TICKS_PER_DEG = %.4f;", ticksPerDeg));
                telemetry.addLine(String.format("TURRET_DIRECTION = %.1f;", direction));
                telemetry.addLine();
                telemetry.addData("With current Shooter values, turret is at", "%.1f deg",
                        Shooter.TURRET_START_DEG + Shooter.TURRET_DIRECTION * ticks / Shooter.TURRET_TICKS_PER_DEG);
            } else {
                telemetry.addLine("Move the turret, measure the angle (right = positive),");
                telemetry.addLine("and type it into MEASURED_DEG to get ticks per degree.");
            }

            // Plain numbers for the Panels graph.
            telemetry.addData("target", TARGET_TICKS);
            telemetry.addData("ticks", ticks);
            telemetry.update();
        }
        turret.setPower(0);
    }
}
