package org.firstinspires.ftc.teamcode.OpModes.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.pedroPathing.Field.Alliance;

/**
 * The match TeleOp for the RED alliance. Everything is in MainTeleOp; this only picks the
 * alliance, which gives the red HIVE cells and the red start pose: the blue one with the field
 * turned 180 deg, (59, 24) at the audience wall facing away (heading 90).
 */
@TeleOp(name = "Main TeleOp RED", group = "TeleOp")
public class MainTeleOpRed extends MainTeleOp {
    @Override
    protected Alliance alliance() {
        return Alliance.RED;
    }
}
