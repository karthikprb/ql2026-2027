package org.firstinspires.ftc.teamcode.OpModes.ComponentTest;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Motor Tester + Servo Tester in one OpMode: one motor and three servos, all from Panels.
 *
 * Panels > Configurables > MotorServoTester:
 *   MOTOR_NAME, POWER                          = motor name (read at init), power -1..1
 *   SERVO_1_NAME .. SERVO_3_NAME               = servo names (read at init)
 *   SERVO_1_POS  .. SERVO_3_POS                = 0..1, written every loop
 *
 * Set the names before pressing init. POWER starts at 0 so the motor doesn't spin on start.
 */
@Configurable
@TeleOp(name = "Motor + Servo Tester", group = "Test")
public class MotorServoTester extends LinearOpMode {

    public static String MOTOR_NAME = "shooterLeft";
    public static double POWER = 0.0;

    public static String SERVO_1_NAME = "servo1";
    public static String SERVO_2_NAME = "servo2";
    public static String SERVO_3_NAME = "servo3";

    public static double SERVO_1_POS = 0.5;
    public static double SERVO_2_POS = 0.5;
    public static double SERVO_3_POS = 0.5;

    @Override
    public void runOpMode() {
        telemetry = new JoinedTelemetry(PanelsTelemetry.INSTANCE.getFtcTelemetry(), telemetry);

        DcMotor motor = hardwareMap.get(DcMotor.class, MOTOR_NAME);
        Servo servo1 = hardwareMap.get(Servo.class, SERVO_1_NAME);
        Servo servo2 = hardwareMap.get(Servo.class, SERVO_2_NAME);
        Servo servo3 = hardwareMap.get(Servo.class, SERVO_3_NAME);

        waitForStart();

        while (opModeIsActive()) {
            motor.setPower(POWER);
            servo1.setPosition(SERVO_1_POS);
            servo2.setPosition(SERVO_2_POS);
            servo3.setPosition(SERVO_3_POS);

            telemetry.addData(MOTOR_NAME + " power", POWER);
            telemetry.addData(MOTOR_NAME + " ticks", motor.getCurrentPosition());
            telemetry.addData(SERVO_1_NAME, SERVO_1_POS);
            telemetry.addData(SERVO_2_NAME, SERVO_2_POS);
            telemetry.addData(SERVO_3_NAME, SERVO_3_POS);
            telemetry.update();
        }
    }
}
