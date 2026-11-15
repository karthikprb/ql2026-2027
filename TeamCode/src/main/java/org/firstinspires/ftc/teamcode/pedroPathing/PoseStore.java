package org.firstinspires.ftc.teamcode.pedroPathing;

import org.firstinspires.ftc.teamcode.follower.math.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.Field.Alliance;

/**
 * Hands the robot's position from the auto to the TeleOp.
 *
 * Static fields live as long as the Robot Controller app does, which is across the auto -> TeleOp
 * switch in a match. The auto calls save() every loop, so whatever pose it ends with (even if it
 * is stopped early) is here when the TeleOp inits. The TeleOp reads it and then clear()s it, so a
 * pose is only ever used once: running the TeleOp again later starts from its default pose, not
 * from a stale auto.
 */
public final class PoseStore {

    private static Pose pose = null;
    private static Alliance alliance = null;   // null = unknown, usable by either TeleOp
    private static long savedAtMs = 0;

    private PoseStore() {}

    /** Remember where the robot is. alliance may be null if the OpMode doesn't know it. */
    public static void save(Pose robotPose, Alliance robotAlliance) {
        pose = robotPose;
        alliance = robotAlliance;
        savedAtMs = System.currentTimeMillis();
    }

    /** True when a pose was saved and it is from this alliance's auto (or from an auto that didn't say). */
    public static boolean hasPoseFor(Alliance teleOpAlliance) {
        return pose != null && (alliance == null || alliance == teleOpAlliance);
    }

    public static Pose pose() {
        return pose;
    }

    public static Alliance alliance() {
        return alliance;
    }

    /** Seconds since the pose was saved, for telemetry. */
    public static double ageSeconds() {
        return pose == null ? 0 : (System.currentTimeMillis() - savedAtMs) / 1000.0;
    }

    public static void clear() {
        pose = null;
        alliance = null;
        savedAtMs = 0;
    }
}
