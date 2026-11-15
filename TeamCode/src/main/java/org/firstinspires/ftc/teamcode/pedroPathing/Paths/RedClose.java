package org.firstinspires.ftc.teamcode.pedroPathing.Paths;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.pedroPathing.Field.Alliance;

/**
 * Red close auto: the mirror image of Blue Close. The whole auto is in CloseAuto; this only
 * picks the alliance, and CloseAuto mirrors every blue pose across the centre line
 * (x -> 144 - x, heading -> 180 - heading). Start (95, 137, 270) becomes (49, 137, 270).
 */
@Autonomous(name = "Red Close")
public class RedClose extends CloseAuto {
    @Override
    protected Alliance alliance() {
        return Alliance.RED;
    }
}
