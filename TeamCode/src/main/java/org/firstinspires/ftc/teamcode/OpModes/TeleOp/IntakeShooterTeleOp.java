package org.firstinspires.ftc.teamcode.OpModes.TeleOp;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Components.Intake;
import org.firstinspires.ftc.teamcode.Components.Shooter;

/**
 * Intake and Shooter together. All hardware names, powers, servo positions, gains and
 * limits come from Intake.java and Shooter.java, so tune them there (or in Panels > Shooter).
 *
 * ---------- Gamepad 1 ----------
 *   A             = intake ON    (Intake INTAKE: dropdown down, motor at INTAKE_POWER)
 *   B             = intake OFF   (Intake STOW: dropdown up, motor off)
 *   X             = intake OUTTAKE (dropdown down, motor at OUTTAKE_POWER)
 *   Right bumper  = shooter ON at SHOOTER_RPM
 *   Left bumper   = shooter OFF
 *
 * ---------- Gamepad 2 (Shooter's own trim controls, from Shooter.handleGamepad) ----------
 *   Dpad up / down      = RPM    +/- Shooter.RPM_STEP
 *   Right / left bumper = hood   +/- Shooter.HOOD_STEP
 *   Dpad right / left   = turret +/- Shooter.TURRET_STEP_DEG
 *   Options             = clear all trims
 *
 * The turret holds wherever it was pointing at INIT (Shooter treats that as TURRET_START_DEG),
 * so point it forward before pressing INIT. The hood starts at Shooter.HOOD_MIN.
 */
@Configurable
@TeleOp(name = "Intake + Shooter", group = "TeleOp")
public class IntakeShooterTeleOp extends LinearOpMode {

    /** Flywheel speed when the right bumper turns the shooter on. Editable in Panels. */
    public static double SHOOTER_RPM = 2200;

    @Override
    public void runOpMode() {
        // Every telemetry line goes to both the Driver Station and Panels.
        telemetry = new JoinedTelemetry(PanelsTelemetry.INSTANCE.getFtcTelemetry(), telemetry);

        Intake intake = new Intake(hardwareMap);
        Shooter shooter = new Shooter(hardwareMap);

        telemetry.addLine("Intake + Shooter ready.");
        telemetry.addLine("Point the turret forward before pressing INIT.");
        telemetry.addLine("GP1: A intake on, B intake off, X outtake, RB shooter on, LB shooter off");
        telemetry.update();
        waitForStart();

        boolean shooterOn = false;

        while (opModeIsActive()) {
            // ---------------- gamepad 1: on / off ----------------
            if (gamepad1.aWasPressed()) intake.setState(Intake.State.INTAKE);
            if (gamepad1.bWasPressed()) intake.setState(Intake.State.STOW);
            if (gamepad1.xWasPressed()) intake.setState(Intake.State.OUTTAKE);

            if (gamepad1.rightBumperWasPressed()) shooterOn = true;
            if (gamepad1.leftBumperWasPressed()) shooterOn = false;
            // Set every loop so a SHOOTER_RPM change in Panels applies straight away.
            shooter.setTargetRpm(shooterOn ? SHOOTER_RPM : 0);

            // ---------------- gamepad 2: shooter trims ----------------
            shooter.handleGamepad(gamepad2);

            shooter.update();

            // ---------------- telemetry ----------------
            telemetry.addData("Intake", intake.getState());
            telemetry.addData("Shooter", shooterOn ? "ON" : "off");
            telemetry.addData("Target RPM (with trim)", "%.0f", shooter.commandedRpm());
            telemetry.addData("Left / Right RPM", "%.0f / %.0f", shooter.getLeftRpm(), shooter.getRightRpm());
            telemetry.addData("At speed", shooter.flywheelAtSpeed());
            telemetry.addData("Hood", "%.3f", shooter.commandedHood());
            telemetry.addData("Turret target / now (deg)", "%.1f / %.1f",
                    shooter.commandedTurretDeg(), shooter.getTurretDegrees());
            telemetry.addData("Trims RPM / hood / turret", "%.0f / %.3f / %.1f",
                    shooter.getRpmTrim(), shooter.getHoodTrim(), shooter.getTurretTrimDeg());
            telemetry.addData("Battery (V) / power scale", "%.2f / %.3f",
                    shooter.getBatteryVoltage(), shooter.getVoltageScale());
            // Plain numbers for the Panels graph.
            telemetry.addData("targetRpm", shooter.commandedRpm());
            telemetry.addData("leftRpm", shooter.getLeftRpm());
            telemetry.addData("rightRpm", shooter.getRightRpm());
            telemetry.update();
        }

        intake.stop();
        shooter.stop();
        shooter.update();
    }
}
