package org.firstinspires.ftc.teamcode.OpModes.ComponentTest;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Wrapper.CacheMotor;
import org.firstinspires.ftc.teamcode.Wrapper.GamepadEx;

@TeleOp
public class motorTest extends LinearOpMode {

    CacheMotor motor;
    GamepadEx gamepadEx;
    Boolean run = false;

    @Override
    public void runOpMode() throws InterruptedException {
        motor = new CacheMotor(hardwareMap,"front_left");
        gamepadEx = new GamepadEx(gamepad1);
        waitForStart();

        while(opModeIsActive()){

            if (gamepadEx.isPress(GamepadEx.Control.a)){
                run = !run;
            }
            if (run){
                motor.setPower(1);
            }else {
                motor.setPower(0);
            }
            gamepadEx.loop();
            motor.write();
        }

    }
}
