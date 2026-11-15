package org.firstinspires.ftc.teamcode.pedroPathing.Paths;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.pedroPathing.Field.Alliance;

/**
 * Red close auto: the same moves as Blue Close with the field turned 180 degrees. Starts at
 * the measured red spot (47, 8, 90) at the audience wall facing away, shoots, drives
 * diagonally to the driver's left to (22, 25), then on to park at (12, 79).
 * The whole auto is in CloseAuto; this only picks the alliance. Edit the red poses in
 * CloseAuto.poses().
 */
@Autonomous(name = "Red Close")
public class RedClose extends CloseAuto {
    @Override
    protected Alliance alliance() {
        return Alliance.RED;
    }
}
