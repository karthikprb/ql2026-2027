package org.firstinspires.ftc.teamcode.OpModes.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.pedroPathing.Field.Alliance;

/** The match TeleOp for the BLUE alliance. Everything is in MainTeleOp; this only picks the alliance. */
@TeleOp(name = "Main TeleOp BLUE", group = "TeleOp")
public class MainTeleOpBlue extends MainTeleOp {
    @Override
    protected Alliance alliance() {
        return Alliance.BLUE;
    }
}
