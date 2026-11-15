package org.firstinspires.ftc.teamcode.OpModes.ComponentTest.shooter;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Components.Shooter;

/**
 * Tunes the turret PID (Shooter.TURRET_KP / KI / KD / KS). Run "Shooter 4: Turret Position"
 * first and paste TURRET_MIN/MAX_DEG and TURRET_MIN/MAX_TICKS into Shooter.java, because the
 * PID works in degrees and needs those to know where the turret is.
 *
 * Point the turret straight forward (TURRET_START_DEG) and press INIT.
 *
 * Gamepad 1:
 *   A          = jump between ANGLE_A and ANGLE_B (a step to tune against)
 *   B          = auto-step: keep jumping between them every STEP_TIME seconds
 *   Dpad up    = go to TURRET_MAX_DEG (clockwise end)
 *   Dpad down  = go to TURRET_MIN_DEG (counterclockwise end)
 *   Y          = back to TURRET_FORWARD_DEG
 *
 * Panels > Shooter: change TURRET_KP / KI / KD / KS / MAX_POWER live, they apply straight away.
 * Panels graph: watch "target" vs "current". Tuning order:
 *   1. KI = KD = KS = 0. Raise KP until it gets to the target quickly; back off if it overshoots
 *      and wobbles.
 *   2. Raise KD to stop the overshoot.
 *   3. If it stops a little short every time, raise KS (the power just needed to start it moving).
 *   4. Only add KI if it still sits short of the target after that.
 * If the turret runs AWAY from the target (error grows), the motor turns the opposite way from
 * its encoder: fix that in the robot config, not with negative gains.
 */
@Configurable
@TeleOp(name = "Shooter 5: Turret PID", group = "Shooter")
public class TurretPidTest extends LinearOpMode {

    public static double ANGLE_A = 80;
    public static double ANGLE_B = 100;
    public static double STEP_TIME = 1.5;   // s between jumps in auto-step

    @Override
    public void runOpMode() {
        telemetry = new JoinedTelemetry(PanelsTelemetry.INSTANCE.getFtcTelemetry(), telemetry);

        Shooter shooter = new Shooter(hardwareMap);   // with the turret; flywheels stay off

        telemetry.addLine("Turret should be pointing FORWARD now. Count resets to 0 here.");
        telemetry.addLine("A = step A/B, B = auto-step, dpad up/down = ends, Y = forward.");
        telemetry.update();
        waitForStart();

        double target = Shooter.TURRET_FORWARD_DEG;
        boolean atB = false;
        boolean autoStep = false;
        ElapsedTime stepTimer = new ElapsedTime();

        while (opModeIsActive()) {
            if (gamepad1.aWasPressed()) {
                atB = !atB;
                target = atB ? ANGLE_B : ANGLE_A;
                autoStep = false;
            }
            if (gamepad1.bWasPressed()) {
                autoStep = !autoStep;
                stepTimer.reset();
            }
            if (gamepad1.dpadUpWasPressed()) { target = Shooter.TURRET_MAX_DEG; autoStep = false; }
            if (gamepad1.dpadDownWasPressed()) { target = Shooter.TURRET_MIN_DEG; autoStep = false; }
            if (gamepad1.yWasPressed()) { target = Shooter.TURRET_FORWARD_DEG; autoStep = false; }

            if (autoStep && stepTimer.seconds() >= STEP_TIME) {
                atB = !atB;
                target = atB ? ANGLE_B : ANGLE_A;
                stepTimer.reset();
            }

            shooter.setTargetRpm(0);
            shooter.setTurretDegrees(target);
            shooter.update();

            telemetry.addData("Mode", autoStep ? "AUTO-STEP (B to stop)" : "manual");
            telemetry.addData("Target / current (deg)", "%.1f / %.1f",
                    shooter.commandedTurretDeg(), shooter.getTurretDegrees());
            telemetry.addData("Error (deg)", "%.2f", shooter.getTurretErrorDeg());
            telemetry.addData("Power", "%.3f", shooter.getTurretPower());
            telemetry.addData("Ticks", shooter.getTurretTicks());
            telemetry.addData("On target", shooter.turretOnTarget());
            telemetry.addData("Gains P / I / D / S", "%.4f / %.4f / %.5f / %.3f",
                    Shooter.TURRET_KP, Shooter.TURRET_KI, Shooter.TURRET_KD, Shooter.TURRET_KS);
            // Plain numbers for the Panels graph.
            telemetry.addData("target", shooter.commandedTurretDeg());
            telemetry.addData("current", shooter.getTurretDegrees());
            telemetry.addData("power", shooter.getTurretPower());
            telemetry.update();
        }
        shooter.stop();
    }
}
