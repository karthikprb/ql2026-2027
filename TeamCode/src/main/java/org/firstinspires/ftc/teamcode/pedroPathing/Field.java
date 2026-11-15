package org.firstinspires.ftc.teamcode.pedroPathing;

import org.firstinspires.ftc.teamcode.follower.math.Pose;
import org.firstinspires.ftc.teamcode.follower.utils.Angle;

/**
 * BIOBUZZ field facts shared by the autos and TeleOps.
 *
 * Frame: inches, origin at the corner, 0..144 on both axes, the same frame the Pedro pose uses.
 *   x = across the field as seen from the audience. Red HIVE cells are on the x < 72 half,
 *       blue cells on the x > 72 half.
 *   y = toward / away from the audience. Audience at y = 0, far wall at y = 144.
 *   Heading 0 = +x, counterclockwise positive. 270 = facing the audience, 90 = facing away.
 *
 * The HIVE frame is in the centre of the field. Each alliance's HIVE has two CELLS about
 * 18.8 in apart, one toward the audience ("near", y = 62.6) and one toward the far wall
 * ("far", y = 81.4).
 *
 * Red is the blue side TURNED 180 DEGREES about the centre of the field, not a left/right
 * mirror: measured on the field, blue's start (95, 137, 270) is red's (47, 8, 90). Blue starts
 * at the far wall facing the audience; red starts at the audience wall facing away. So:
 *     x -> 144 - x,  y -> 144 - y,  heading -> heading + 180
 * forAlliance() does that. If a measured red spot differs by an inch or two from the turned
 * blue one (as the start does), write the red pose out explicitly instead, like CloseAuto does.
 */
public final class Field {

    public enum Alliance { BLUE, RED }

    public static final double LENGTH_IN = 144;
    public static final double CENTER_X = 72;
    public static final double CENTER_Y = 72;

    private Field() {}

    /** The red-side copy of a blue-side pose: the field turned 180 degrees about its centre. */
    public static Pose toRed(Pose blue) {
        return new Pose(LENGTH_IN - blue.x(), LENGTH_IN - blue.y(), Angle.normalize(blue.heading() + Math.PI));
    }

    /** Same spot for the given alliance: unchanged for blue, turned 180 degrees for red. */
    public static Pose forAlliance(Alliance alliance, Pose blue) {
        return alliance == Alliance.BLUE ? blue : toRed(blue);
    }

    /** Turn a heading in degrees to the other alliance's (adds 180). */
    public static double toRedHeadingDeg(double headingDeg) {
        double h = headingDeg + 180;
        while (h < 0) h += 360;
        while (h >= 360) h -= 360;
        return h;
    }
}
