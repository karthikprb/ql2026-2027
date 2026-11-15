package org.firstinspires.ftc.teamcode.pedroPathing;

import org.firstinspires.ftc.teamcode.follower.math.Pose;
import org.firstinspires.ftc.teamcode.follower.utils.Angle;

/**
 * BIOBUZZ field facts shared by the autos and TeleOps.
 *
 * Frame: inches, origin at the corner, 0..144 on both axes, the same frame the Pedro pose uses.
 *   x = across the field as seen from the audience. Red is the LEFT half (field columns A-C,
 *       x < 72), blue is the RIGHT half (columns D-F, x > 72). (Competition Manual 9.1 / G304)
 *   y = toward / away from the audience. Audience at y = 0, far wall at y = 144.
 *   Heading 0 = +x, counterclockwise positive. 270 deg = facing the audience (-y).
 *
 * The HIVE frame is in the centre of the field. It holds a red HIVE (red half) and a blue HIVE
 * (blue half); each has two CELLS about 18.8 in apart, one toward the audience ("near") and one
 * toward the far wall ("far"). So the red side is the MIRROR IMAGE of the blue side across the
 * centre line x = 72: x -> 144 - x, y stays, heading -> 180 - heading.
 *
 * Write everything once for BLUE and get the red version from forAlliance(). If a measured
 * red number ever disagrees with the mirror, change mirror() here, nowhere else.
 */
public final class Field {

    public enum Alliance { BLUE, RED }

    public static final double LENGTH_IN = 144;
    public static final double CENTER_X = 72;   // the red / blue dividing line
    public static final double CENTER_Y = 72;

    private Field() {}

    /** The red-side copy of a blue-side pose: x mirrored across the centre line, heading flipped left/right. */
    public static Pose mirror(Pose blue) {
        return new Pose(LENGTH_IN - blue.x(), blue.y(), mirrorHeading(blue.heading()));
    }

    /** Same point / heading for the given alliance: unchanged for blue, mirrored for red. */
    public static Pose forAlliance(Alliance alliance, Pose blue) {
        return alliance == Alliance.BLUE ? blue : mirror(blue);
    }

    /** Mirror an x coordinate across the centre line. */
    public static double mirrorX(double x) {
        return LENGTH_IN - x;
    }

    /** Mirror a heading (radians) across the centre line: 0 <-> 180, 90 and 270 stay put. */
    public static double mirrorHeading(double headingRad) {
        return Angle.normalize(Math.PI - headingRad);
    }

    /** Mirror a heading in degrees across the centre line. */
    public static double mirrorHeadingDeg(double headingDeg) {
        double h = 180 - headingDeg;
        while (h < 0) h += 360;
        while (h >= 360) h -= 360;
        return h;
    }
}
