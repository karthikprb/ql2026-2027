package org.firstinspires.ftc.teamcode.OpModes.ComponentTest;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Sends three servos to the positions set in Panels. Nothing else.
 *
 * Panels > Configurables > ServoTester:
 *   SERVO_1_NAME, SERVO_2_NAME, SERVO_3_NAME  = hardware map names (read at init)
 *   SERVO_1_POS,  SERVO_2_POS,  SERVO_3_POS   = 0..1, written to the servo every loop
 *
 * Change a position in Panels and the servo goes there. Set the names before pressing init.
 */
@Configurable
@TeleOp(name = "Servo Tester", group = "Test")
public class ServoTester extends LinearOpMode {

    public static String SERVO_1_NAME = "servo1";
    public static String SERVO_2_NAME = "servo2";
    public static String SERVO_3_NAME = "servo3";

    public static double SERVO_1_POS = 0.5;
    public static double SERVO_2_POS = 0.5;
    public static double SERVO_3_POS = 0.5;

    @Override
    public void runOpMode() {
        telemetry = new JoinedTelemetry(PanelsTelemetry.INSTANCE.getFtcTelemetry(), telemetry);

        Servo servo1 = hardwareMap.get(Servo.class, SERVO_1_NAME);
        Servo servo2 = hardwareMap.get(Servo.class, SERVO_2_NAME);
        Servo servo3 = hardwareMap.get(Servo.class, SERVO_3_NAME);

        waitForStart();

        while (opModeIsActive()) {
            servo1.setPosition(SERVO_1_POS);
            servo2.setPosition(SERVO_2_POS);
            servo3.setPosition(SERVO_3_POS);

            telemetry.addData(SERVO_1_NAME, SERVO_1_POS);
            telemetry.addData(SERVO_2_NAME, SERVO_2_POS);
            telemetry.addData(SERVO_3_NAME, SERVO_3_POS);
            telemetry.update();
        }
    }
}
