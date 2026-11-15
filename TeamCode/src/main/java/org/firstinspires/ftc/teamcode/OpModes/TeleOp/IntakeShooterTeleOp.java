package org.firstinspires.ftc.teamcode.OpModes.TeleOp;

import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Components.Drive;
import org.firstinspires.ftc.teamcode.Components.Intake;
import org.firstinspires.ftc.teamcode.Components.Shooter;
import org.firstinspires.ftc.teamcode.follower.api.PoseFactory;
import org.firstinspires.ftc.teamcode.follower.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

/**
 * Driving, Intake and Shooter together.
 *
 * This OpMode has no gamepad code of its own: every button is handled inside the component
 * it controls, so change controls (and their settings) there:
 *   Drive.handleGamepad(gamepad1)          sticks, right trigger slow mode, Options field-centric
 *   Intake.handleGamepad(gamepad1)         A intake, B stow, X outtake
 *   Shooter.handleGamepad(gamepad1)        RB shooter on, LB shooter off, Y hood up / down
 *   Shooter.handleTrimGamepad(gamepad2)    dpad up / down RPM trim, Options clear trims
 *
 * The turret is OFF in this OpMode: Shooter is made without it, so the turret motor is never
 * accessed and doesn't need to be in the robot config.
 * (TURRET COMMENTED OUT FOR NOW: the turret holds wherever it was pointing at INIT, since
 *  Shooter treats that as TURRET_START_DEG, so point it forward before pressing INIT.)
 */
@TeleOp(name = "Drive + Intake + Shooter", group = "TeleOp")
public class IntakeShooterTeleOp extends LinearOpMode {

    @Override
    public void runOpMode() {
        // Every telemetry line goes to both the Driver Station and Panels.
        telemetry = new JoinedTelemetry(PanelsTelemetry.INSTANCE.getFtcTelemetry(), telemetry);

        Follower follower = Constants.create(hardwareMap);
        Drive drive = new Drive(follower);
        Intake intake = new Intake(hardwareMap);
        // TURRET COMMENTED OUT FOR NOW: false = Shooter never looks up or drives the turret motor.
        // Switch back to new Shooter(hardwareMap) to use the turret again.
        Shooter shooter = new Shooter(hardwareMap, false);

        telemetry.addLine("Drive + Intake + Shooter ready.");
        // TURRET COMMENTED OUT FOR NOW
        // telemetry.addLine("Point the turret forward before pressing INIT.");
        telemetry.addLine("GP1: sticks drive, RT slow, Options field-centric");
        telemetry.addLine("GP1: A intake on, B intake off, X outtake, Y hood, RB shooter on, LB shooter off");
        telemetry.addLine("GP2: dpad up/down RPM trim, Options clear trims");
        telemetry.update();
        waitForStart();

        // Start the odometry at 0 heading so field-centric "forward" = the way the robot faces now.
        follower.setPose(PoseFactory.degrees().of(0, 0, 0));
        follower.manual(0, 0, 0);

        while (opModeIsActive()) {
            follower.update();

            drive.handleGamepad(gamepad1);
            intake.handleGamepad(gamepad1);
            shooter.handleGamepad(gamepad1);
            shooter.handleTrimGamepad(gamepad2);

            shooter.update();

            // ---------------- telemetry ----------------
            telemetry.addData("Drive", (drive.isFieldCentric() ? "FIELD-centric" : "ROBOT-centric")
                    + (drive.isSlow() ? "  SLOW" : "") + "  (Options to switch)");
            telemetry.addData("Heading (deg)", "%.1f", Math.toDegrees(follower.pose().heading()));
            telemetry.addData("Intake", intake.getState());
            telemetry.addData("Shooter", shooter.isShooterOn() ? "ON" : "off");
            telemetry.addData("Target RPM (with trim)", "%.0f", shooter.commandedRpm());
            telemetry.addData("Left / Right RPM", "%.0f / %.0f", shooter.getLeftRpm(), shooter.getRightRpm());
            telemetry.addData("At speed", shooter.flywheelAtSpeed());
            telemetry.addData("Hood", "%s  %.3f  (Y to toggle)", shooter.isHoodUp() ? "UP" : "down", shooter.commandedHood());
            // TURRET COMMENTED OUT FOR NOW
            // telemetry.addData("Turret target / now (deg)", "%.1f / %.1f",
            //         shooter.commandedTurretDeg(), shooter.getTurretDegrees());
            // telemetry.addData("Trims RPM / turret", "%.0f / %.1f",
            //         shooter.getRpmTrim(), shooter.getTurretTrimDeg());
            telemetry.addData("RPM trim", "%.0f", shooter.getRpmTrim());
            // Plain numbers for the Panels graph.
            telemetry.addData("targetRpm", shooter.commandedRpm());
            telemetry.addData("leftRpm", shooter.getLeftRpm());
            telemetry.addData("rightRpm", shooter.getRightRpm());
            telemetry.update();
        }

        drive.stop();
        intake.stop();
        shooter.stop();
        shooter.update();
    }
}
