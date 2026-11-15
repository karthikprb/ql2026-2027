package org.firstinspires.ftc.teamcode.OpModes.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.pedroPathing.Field.Alliance;

/**
 * The match TeleOp for the RED alliance. Everything is in MainTeleOp; this only picks the
 * alliance, which gives the red HIVE cells and the mirrored start pose (x -> 144 - x).
 */
@TeleOp(name = "Main TeleOp RED", group = "TeleOp")
public class MainTeleOpRed extends MainTeleOp {
    @Override
    protected Alliance alliance() {
        return Alliance.RED;
    }
}
