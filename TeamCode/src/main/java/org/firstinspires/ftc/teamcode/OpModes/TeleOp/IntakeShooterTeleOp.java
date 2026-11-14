package org.firstinspires.ftc.teamcode.OpModes.TeleOp;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Components.Intake;
import org.firstinspires.ftc.teamcode.Components.Shooter;
import org.firstinspires.ftc.teamcode.follower.api.PoseFactory;
import org.firstinspires.ftc.teamcode.follower.drivetrain.DrivePowers;
import org.firstinspires.ftc.teamcode.follower.follower.Follower;
import org.firstinspires.ftc.teamcode.follower.follower.ManualDrive;
import org.firstinspires.ftc.teamcode.follower.math.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

/**
 * Driving, Intake and Shooter together. All hardware names, powers, servo positions, gains
 * and limits come from Constants (drive), Intake.java and Shooter.java, so tune them there
 * (or in Panels > Shooter).
 *
 * ---------- Gamepad 1 ----------
 *   Left stick    = drive / strafe
 *   Right stick X = turn
 *   Right trigger = hold for slow mode
 *   Options       = toggle field-centric driving (forward = the way the robot faced at START)
 *   A             = intake ON    (Intake INTAKE: dropdown down, motor at INTAKE_POWER)
 *   B             = intake OFF   (Intake STOW: dropdown up, motor off)
 *   X             = intake OUTTAKE (dropdown down, motor at OUTTAKE_POWER)
 *   Right bumper  = shooter ON at SHOOTER_RPM
 *   Left bumper   = shooter OFF
 *
 * ---------- Gamepad 2 (Shooter's own trim controls, from Shooter.handleGamepad) ----------
 *   Dpad up / down      = RPM    +/- Shooter.RPM_STEP
 *   Right / left bumper = hood   +/- Shooter.HOOD_STEP
 *   Options             = clear all trims
 *   (TURRET COMMENTED OUT FOR NOW: Dpad right / left = turret +/- Shooter.TURRET_STEP_DEG)
 *
 * The hood starts at Shooter.HOOD_MIN.
 * The turret is OFF in this OpMode: Shooter is made without it, so the turret motor is never
 * accessed and doesn't need to be in the robot config.
 * (TURRET COMMENTED OUT FOR NOW: the turret holds wherever it was pointing at INIT, since
 *  Shooter treats that as TURRET_START_DEG, so point it forward before pressing INIT.)
 */
@Configurable
@TeleOp(name = "Drive + Intake + Shooter", group = "TeleOp")
public class IntakeShooterTeleOp extends LinearOpMode {

    /** Flywheel speed when the right bumper turns the shooter on. Editable in Panels. */
    public static double SHOOTER_RPM = 2200;

    /** Drive power multiplier while the right trigger is held. Editable in Panels. */
    public static double SLOW_MODE_SCALE = 0.4;

    @Override
    public void runOpMode() {
        // Every telemetry line goes to both the Driver Station and Panels.
        telemetry = new JoinedTelemetry(PanelsTelemetry.INSTANCE.getFtcTelemetry(), telemetry);

        Follower follower = Constants.create(hardwareMap);
        Intake intake = new Intake(hardwareMap);
        // TURRET COMMENTED OUT FOR NOW: false = Shooter never looks up or drives the turret motor.
        // Switch back to new Shooter(hardwareMap) to use the turret again.
        Shooter shooter = new Shooter(hardwareMap, false);

        telemetry.addLine("Drive + Intake + Shooter ready.");
        // TURRET COMMENTED OUT FOR NOW
        // telemetry.addLine("Point the turret forward before pressing INIT.");
        telemetry.addLine("GP1: sticks drive, RT slow, Options field-centric");
        telemetry.addLine("GP1: A intake on, B intake off, X outtake, RB shooter on, LB shooter off");
        telemetry.update();
        waitForStart();

        // Start the odometry at 0 heading so field-centric "forward" = the way the robot faces now.
        follower.setPose(PoseFactory.degrees().of(0, 0, 0));
        follower.manual(0, 0, 0);

        boolean shooterOn = false;
        boolean fieldCentric = false;

        while (opModeIsActive()) {
            follower.update();
            Pose pose = follower.pose();

            // ---------------- gamepad 1: driving ----------------
            if (gamepad1.optionsWasPressed()) fieldCentric = !fieldCentric;

            double scale = gamepad1.right_trigger > 0.5 ? SLOW_MODE_SCALE : 1.0;
            DrivePowers powers = new DrivePowers(
                    -gamepad1.left_stick_y * scale,    // forward
                    -gamepad1.left_stick_x * scale,    // strafe (left = positive)
                    -gamepad1.right_stick_x * scale);  // turn (left = positive)
            if (fieldCentric) powers = ManualDrive.fieldCentric(powers, pose.heading());
            follower.manual(powers);

            // ---------------- gamepad 1: intake / shooter ----------------
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
            telemetry.addData("Drive", (fieldCentric ? "FIELD-centric" : "ROBOT-centric")
                    + (scale < 1.0 ? "  SLOW" : "") + "  (Options to switch)");
            telemetry.addData("Heading (deg)", "%.1f", Math.toDegrees(pose.heading()));
            telemetry.addData("Intake", intake.getState());
            telemetry.addData("Shooter", shooterOn ? "ON" : "off");
            telemetry.addData("Target RPM (with trim)", "%.0f", shooter.commandedRpm());
            telemetry.addData("Left / Right RPM", "%.0f / %.0f", shooter.getLeftRpm(), shooter.getRightRpm());
            telemetry.addData("At speed", shooter.flywheelAtSpeed());
            telemetry.addData("Hood", "%.3f", shooter.commandedHood());
            // TURRET COMMENTED OUT FOR NOW
            // telemetry.addData("Turret target / now (deg)", "%.1f / %.1f",
            //         shooter.commandedTurretDeg(), shooter.getTurretDegrees());
            // telemetry.addData("Trims RPM / hood / turret", "%.0f / %.3f / %.1f",
            //         shooter.getRpmTrim(), shooter.getHoodTrim(), shooter.getTurretTrimDeg());
            telemetry.addData("Trims RPM / hood", "%.0f / %.3f",
                    shooter.getRpmTrim(), shooter.getHoodTrim());
            // Plain numbers for the Panels graph.
            telemetry.addData("targetRpm", shooter.commandedRpm());
            telemetry.addData("leftRpm", shooter.getLeftRpm());
            telemetry.addData("rightRpm", shooter.getRightRpm());
            telemetry.update();
        }

        follower.stop();
        follower.update();
        intake.stop();
        shooter.stop();
        shooter.update();
    }
}
