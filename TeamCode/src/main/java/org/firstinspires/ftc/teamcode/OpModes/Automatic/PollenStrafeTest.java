package org.firstinspires.ftc.teamcode.OpModes.Automatic;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.follower.drivetrain.DrivePowers;
import org.firstinspires.ftc.teamcode.follower.hardware.Mecanum;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name="Pollen Align Test",group="Testing")
public class PollenStrafeTest extends LinearOpMode{
    Mecanum mecanum;
    Limelight3A limelight;

    @Override
    public void runOpMode() throws InterruptedException{
        mecanum=new Mecanum(hardwareMap,Constants.drivetrainConfig);
        limelight=hardwareMap.get(Limelight3A.class,"limelight");
        // green ball
        limelight.pipelineSwitch(1);
        limelight.start();
        telemetry.addLine("Ready");
        telemetry.update();

        waitForStart();

        while(opModeIsActive()){
            LLResult result=limelight.getLatestResult();
            if(result!=null&&result.isValid()){
                double tx=result.getTx();
                telemetry.addData("Pollen","found");
                telemetry.addData("TX",tx);
                // ball one side, move forward
                if(tx>1){
                    drive(0.25);
                    telemetry.addLine("Moving forward");
                }
                // ball other side, move backward
                else if(tx<-1){
                    drive(-0.25);
                    telemetry.addLine("Moving backward");
                }
                // camera lined up with ball
                else{
                    drive(0);
                    telemetry.addLine("aligned");
                }
            }
            else{
                drive(0);
                telemetry.addLine("No pollen");
            }
            telemetry.update();
        }
        mecanum.stop();
        limelight.stop();
    }

    /** Pedro's Mecanum: positive = forward. */
    private void drive(double forward){
        mecanum.drive(new DrivePowers(forward,0,0),true);
    }
}
