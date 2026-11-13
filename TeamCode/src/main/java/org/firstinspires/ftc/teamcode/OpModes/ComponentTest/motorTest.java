package org.firstinspires.ftc.teamcode.OpModes.ComponentTest;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Runs one motor at the power set in Panels. Nothing else.
 *
 * Panels > Configurables > motorTest:
 *   MOTOR_NAME = hardware map name (read at init)
 *   POWER      = -1..1, written to the motor every loop
 *
 * Set the name before pressing init. Change POWER in Panels and the motor follows.
 * POWER starts at 0 so the motor doesn't spin the moment you press start.
 */
@Configurable
@TeleOp(name = "Motor Tester", group = "Test")
public class motorTest extends LinearOpMode {

    public static String MOTOR_NAME = "shooterLeft";
    public static double POWER = 0.0;

    @Override
    public void runOpMode() {
        telemetry = new JoinedTelemetry(PanelsTelemetry.INSTANCE.getFtcTelemetry(), telemetry);

        DcMotor motor = hardwareMap.get(DcMotor.class, MOTOR_NAME);

        waitForStart();

        while (opModeIsActive()) {
            motor.setPower(POWER);

            telemetry.addData(MOTOR_NAME + " power", POWER);
            telemetry.addData(MOTOR_NAME + " ticks", motor.getCurrentPosition());
            telemetry.update();
        }
    }
}
