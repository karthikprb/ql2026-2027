package org.firstinspires.ftc.teamcode.OpModes.ComponentTest.shooter;

import com.bylazar.gamepad.PanelsGamepad;
import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Components.Shooter;

/**
 * Finds HOOD_MIN and HOOD_MAX: the servo positions at each end of the hood's real travel.
 *
 * Move the hood slowly toward one end. Stop just before it hits the hard stop or the
 * servo starts buzzing, and save it. Do the same at the other end.
 *
 * The hood starts at HOOD_MIN when the Shooter is made, so HOOD_MIN should be the closed end.
 *
 * Gamepad 1:
 *   Dpad up / down    = position +/- 0.005  (fine)
 *   Bumpers           = position +/- 0.05   (coarse, right = up)
 *   A                 = save as the DOWN end   (HOOD_MIN)
 *   B                 = save as the UP end     (HOOD_MAX)
 *   X / Y             = go to the saved down / up end to double-check
 *
 * The servo doesn't move until you press something after start.
 */
@TeleOp(name = "Shooter 3: Hood Range", group = "Shooter")
public class HoodRangeTest extends LinearOpMode {

    @Override
    public void runOpMode() {
        // Every telemetry line goes to both the Driver Station and Panels.
        telemetry = new JoinedTelemetry(PanelsTelemetry.INSTANCE.getFtcTelemetry(), telemetry);
        // Gamepad works from the Driver Station or a controller plugged into the Panels computer.
        PanelsPad pad1 = new PanelsPad(gamepad1, PanelsGamepad.INSTANCE.getFirstManager());

        Servo hood = hardwareMap.get(Servo.class, Shooter.HOOD_NAME);

        telemetry.addLine("Shooter 3: Hood Range. Hood won't move until you press a button.");
        telemetry.update();
        waitForStart();

        double position = (Shooter.HOOD_MIN + Shooter.HOOD_MAX) / 2.0;
        boolean moved = false;
        double down = Double.NaN, up = Double.NaN;

        while (opModeIsActive()) {
            pad1.update();
            double before = position;
            if (pad1.dpadUpWasPressed()) position += 0.005;
            if (pad1.dpadDownWasPressed()) position -= 0.005;
            if (pad1.rightBumperWasPressed()) position += 0.05;
            if (pad1.leftBumperWasPressed()) position -= 0.05;
            if (pad1.aWasPressed()) down = position;
            if (pad1.bWasPressed()) up = position;
            if (pad1.xWasPressed() && !Double.isNaN(down)) position = down;
            if (pad1.yWasPressed() && !Double.isNaN(up)) position = up;
            position = Range.clip(position, 0, 1);

            if (position != before) moved = true;
            if (moved) hood.setPosition(position);

            telemetry.addData("Servo position", moved ? String.format("%.3f", position) : "not moved yet");
            telemetry.addData("Saved down end (A)", Double.isNaN(down) ? "-" : String.format("%.3f", down));
            telemetry.addData("Saved up end (B)", Double.isNaN(up) ? "-" : String.format("%.3f", up));
            telemetry.addLine();

            if (!Double.isNaN(down) && !Double.isNaN(up)) {
                if (down != up) {
                    // Either order is fine: Shooter.setHoodPosition() keeps the hood between the two.
                    telemetry.addLine("Paste into Shooter.java:");
                    telemetry.addLine(String.format("HOOD_MIN = %.3f;   (closed)", down));
                    telemetry.addLine(String.format("HOOD_MAX = %.3f;   (open)", up));
                    telemetry.addLine("and into IntakeShooterTeleOp.java:");
                    telemetry.addLine(String.format("HOOD_CLOSED = %.3f;", down));
                    telemetry.addLine(String.format("HOOD_OPEN = %.3f;", up));
                } else {
                    telemetry.addLine("Both ends are the same position. Save two different ends.");
                }
            }
            telemetry.update();
        }
    }
}
