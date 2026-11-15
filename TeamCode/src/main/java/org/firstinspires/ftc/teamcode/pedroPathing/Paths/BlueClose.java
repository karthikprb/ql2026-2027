package org.firstinspires.ftc.teamcode.pedroPathing.Paths;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.pedroPathing.Field.Alliance;

/** Blue close auto. The whole auto is in CloseAuto; this only picks the alliance. */
@Autonomous(name = "Blue Close")
public class BlueClose extends CloseAuto {
    @Override
    protected Alliance alliance() {
        return Alliance.BLUE;
    }
}
