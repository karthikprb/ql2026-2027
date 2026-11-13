package org.firstinspires.ftc.teamcode.Components;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Wrapper.PidController;
import org.firstinspires.ftc.teamcode.follower.math.Pose;
import org.firstinspires.ftc.teamcode.follower.utils.Angle;

/**
 * Shooter = two flywheel motors (velocity PID + feedforward), a hood servo and a turret motor
 * (position PID, 0..180 deg, no wrapping).
 *
 * Two ways to use it:
 *   - Manual:  setTargetRpm(), setHoodPosition(), setTurretDegrees()
 *   - Auto:    setAlliance() once, then aimAt(robotPose) every loop. It picks the alliance's
 *              goal on the robot's side of the field, and turns distance and angle to it into
 *              an RPM, hood position and turret angle with the placeholder formulas below.
 *
 * In both cases call update() once per loop so the PIDs run, and handleGamepad(gamepad2)
 * if you want the driver to be able to trim RPM / hood / turret with the dpad and bumpers.
 * The trims are added on top of whatever aimAt() or the setters asked for.
 *
 * @Configurable: every public static number below shows up under "Shooter" in Panels and
 * can be changed while an OpMode runs. The PID gains are re-read every update(), so a
 * change takes effect straight away. Panels changes are NOT saved: copy the numbers you
 * like back into this file. (Motor directions and names only apply at the next init.)
 */
@Configurable
public class Shooter {

    // =================================================================
    //  HARDWARE CONFIG NAMES
    // =================================================================
    public static String LEFT_FLYWHEEL_NAME = "shooterLeft";
    public static String RIGHT_FLYWHEEL_NAME = "shooterRight";
    public static String HOOD_NAME = "hood";
    public static String TURRET_NAME = "turret";

    // =================================================================
    //  FLYWHEEL
    // =================================================================
    // 6000 RPM motor (28 ticks per motor rev), geared 1:1 to the flywheel, so one flywheel
    // rev = one motor rev = 28 ticks. Free speed 6000 RPM.
    // If the gearing changes: ticks = 28 * (flywheel gear teeth / motor gear teeth).
    public static double FLYWHEEL_TICKS_PER_REV = 28;
    public static double FLYWHEEL_MAX_RPM = 6000;          // free speed; set from "Shooter 1: Flywheel kF"
    public static double FLYWHEEL_KP = 0.0003;             // power per RPM of error
    public static double FLYWHEEL_KI = 0.0;
    public static double FLYWHEEL_KD = 0.0;
    public static double FLYWHEEL_KF = 1.0 / 6000.0;       // feedforward: power per RPM (~1/free-speed RPM to start)
    public static double RPM_TOLERANCE = 75;               // "at speed" when both wheels are within this
    public static boolean RIGHT_FLYWHEEL_REVERSED = true;  // the two wheels usually spin opposite ways

    // =================================================================
    //  HOOD
    // =================================================================
    public static double HOOD_MIN = 0.0;
    public static double HOOD_MAX = 1.0;

    // =================================================================
    //  TURRET  (0 deg = full left, 90 deg = straight ahead, 180 deg = full right)
    // =================================================================
    public static double TURRET_MIN_DEG = 0;
    public static double TURRET_MAX_DEG = 180;
    public static double TURRET_FORWARD_DEG = 90;          // turret angle that points along the robot's heading
    public static double TURRET_START_DEG = 90;            // where the turret physically is when the OpMode inits
    public static double TURRET_TICKS_PER_DEG = 537.7 * 2.0 / 360.0;  // motor ticks per rev * gear ratio / 360 (PLACEHOLDER)
    public static double TURRET_DIRECTION = 1.0;           // flip to -1 if positive power moves the turret toward 0 deg
    public static double TURRET_KP = 0.02;                 // power per degree of error
    public static double TURRET_KI = 0.0;
    public static double TURRET_KD = 0.001;
    public static double TURRET_MAX_POWER = 0.6;
    public static double TURRET_TOLERANCE_DEG = 1.0;

    // =================================================================
    //  GOALS  (field inches, same frame as the Pedro pose: 0..144 on both axes)
    //
    //  BIOBUZZ: one HIVE structure in the middle of the field holds a red HIVE and a blue
    //  HIVE. Each HIVE has two CELLS, one toward the audience and one toward the far side,
    //  about 18.8 in apart. That is 4 goals total, 2 per alliance. The robot shoots at the
    //  CELL on its own side of the field, so aimAt() picks whichever of the alliance's two
    //  CELLS is closer to the robot.
    //
    //  The x/y/z below are PLACEHOLDERS worked out from the frame size. Measure the real
    //  ones off the field CAD or with a tape measure from the field walls.
    // =================================================================

    /** A goal position on the field: x, y in inches on the tiles, z = opening height above the tiles. */
    public static class Goal {
        public final String name;
        public final double x, y, z;

        public Goal(String name, double x, double y, double z) {
            this.name = name;
            this.x = x;
            this.y = y;
            this.z = z;
        }
    }

    public enum Alliance { RED, BLUE }

    public static double GOAL_Z = 43.95;              // PLACEHOLDER: HIVE pivot axis height. Measure the up-facing CELL opening.
    public static double SHOOTER_EXIT_HEIGHT = 12.0;  // PLACEHOLDER: height of the ball leaving the shooter, inches above tiles

    // Field center is (72, 72). Blue HIVE is on the +x half, red on the -x half (PLACEHOLDER).
    // "Far" = away from the audience (+y), "Near" = audience side (-y).
    public static Goal BLUE_FAR  = new Goal("Blue far",  84, 81.4, GOAL_Z);
    public static Goal BLUE_NEAR = new Goal("Blue near", 84, 62.6, GOAL_Z);
    public static Goal RED_FAR   = new Goal("Red far",   60, 81.4, GOAL_Z);
    public static Goal RED_NEAR  = new Goal("Red near",  60, 62.6, GOAL_Z);

    // PLACEHOLDER formulas, linear in horizontal distance. Replace with your tuned fit (see testing notes).
    //   rpm  = RPM_BASE  + RPM_PER_INCH  * distance
    //   hood = HOOD_BASE + HOOD_PER_INCH * distance
    public static double RPM_BASE = 2500;
    public static double RPM_PER_INCH = 10;
    public static double HOOD_BASE = 0.3;
    public static double HOOD_PER_INCH = 0.002;

    // =================================================================
    //  GAMEPAD 2 TRIM STEPS
    // =================================================================
    public static double RPM_STEP = 50;
    public static double HOOD_STEP = 0.02;
    public static double TURRET_STEP_DEG = 2;

    // ---------------- hardware ----------------
    private final DcMotorEx leftFlywheel;
    private final DcMotorEx rightFlywheel;
    private final Servo hood;
    private final DcMotorEx turret;

    private final PidController leftPid = new PidController(FLYWHEEL_KP, FLYWHEEL_KI, FLYWHEEL_KD);
    private final PidController rightPid = new PidController(FLYWHEEL_KP, FLYWHEEL_KI, FLYWHEEL_KD);
    private final PidController turretPid = new PidController(TURRET_KP, TURRET_KI, TURRET_KD);
    private final ElapsedTime loopTimer = new ElapsedTime();

    // ---------------- targets (before trim) ----------------
    private double targetRpm = 0;
    private double targetHood = HOOD_MIN;
    private double targetTurretDeg = TURRET_START_DEG;

    // ---------------- driver trims (gamepad 2) ----------------
    private double rpmTrim = 0;
    private double hoodTrim = 0;
    private double turretTrimDeg = 0;

    // ---------------- aiming ----------------
    private Alliance alliance = Alliance.BLUE;
    private Goal activeGoal = BLUE_FAR;
    private double lastDistanceToGoal = 0;

    public Shooter(HardwareMap hardwareMap) {
        leftFlywheel = hardwareMap.get(DcMotorEx.class, LEFT_FLYWHEEL_NAME);
        rightFlywheel = hardwareMap.get(DcMotorEx.class, RIGHT_FLYWHEEL_NAME);
        hood = hardwareMap.get(Servo.class, HOOD_NAME);
        turret = hardwareMap.get(DcMotorEx.class, TURRET_NAME);

        // We run our own PID, so the SDK's built-in velocity control is turned off.
        leftFlywheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFlywheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftFlywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightFlywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightFlywheel.setDirection(RIGHT_FLYWHEEL_REVERSED
                ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);

        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftPid.setOutputLimits(-1, 1);
        rightPid.setOutputLimits(-1, 1);
        turretPid.setOutputLimits(-TURRET_MAX_POWER, TURRET_MAX_POWER);

        hood.setPosition(targetHood);
        loopTimer.reset();
    }

    // =================================================================
    //  MANUAL SETTERS
    // =================================================================

    /** Flywheel speed in RPM. 0 turns the flywheels off. */
    public void setTargetRpm(double rpm) {
        targetRpm = Range.clip(rpm, 0, FLYWHEEL_MAX_RPM);
    }

    /** Hood servo position, clipped to HOOD_MIN..HOOD_MAX. */
    public void setHoodPosition(double position) {
        targetHood = Range.clip(position, HOOD_MIN, HOOD_MAX);
    }

    /** Turret angle in degrees, clipped to TURRET_MIN_DEG..TURRET_MAX_DEG. */
    public void setTurretDegrees(double degrees) {
        targetTurretDeg = Range.clip(degrees, TURRET_MIN_DEG, TURRET_MAX_DEG);
    }

    /** Flywheels off, hood down, turret to the forward position. */
    public void stop() {
        setTargetRpm(0);
        setHoodPosition(HOOD_MIN);
        setTurretDegrees(TURRET_FORWARD_DEG);
    }

    // =================================================================
    //  AUTO AIM
    // =================================================================

    /** Which alliance's HIVE to shoot at. Set this once from the OpMode (e.g. in init). */
    public void setAlliance(Alliance alliance) {
        this.alliance = alliance;
    }

    public Alliance getAlliance() {
        return alliance;
    }

    /** The goal aimAt() last used. */
    public Goal getActiveGoal() {
        return activeGoal;
    }

    /**
     * Picks the alliance's CELL on the robot's side of the field, i.e. whichever of the
     * two is closer to the robot.
     */
    public Goal chooseGoal(Pose robotPose) {
        Goal far = alliance == Alliance.BLUE ? BLUE_FAR : RED_FAR;
        Goal near = alliance == Alliance.BLUE ? BLUE_NEAR : RED_NEAR;
        return horizontalDistance(robotPose, far) <= horizontalDistance(robotPose, near) ? far : near;
    }

    /**
     * Works out RPM, hood and turret angle from the robot's field pose and the goal on the
     * robot's side of the field. Call every loop while lining up, then update().
     */
    public void aimAt(Pose robotPose) {
        aimAt(robotPose, chooseGoal(robotPose));
    }

    /** Same as aimAt(pose) but at a specific goal. */
    public void aimAt(Pose robotPose, Goal goal) {
        activeGoal = goal;
        double dx = goal.x - robotPose.x();
        double dy = goal.y - robotPose.y();
        lastDistanceToGoal = Math.hypot(dx, dy);

        setTargetRpm(rpmForDistance(lastDistanceToGoal));
        setHoodPosition(hoodForDistance(lastDistanceToGoal));

        // Angle from the robot to the goal, relative to where the robot is facing.
        // Positive = goal is to the robot's left (CCW), like the Pedro heading.
        double angleToGoal = Math.atan2(dy, dx);
        double relativeRad = Angle.normalizeSigned(angleToGoal - robotPose.heading());
        setTurretDegrees(turretForRelativeAngle(Math.toDegrees(relativeRad)));
    }

    private static double horizontalDistance(Pose robotPose, Goal goal) {
        return Math.hypot(goal.x - robotPose.x(), goal.y - robotPose.y());
    }

    /** How far the goal opening is above the ball's exit point. Same for every goal, since GOAL_Z is shared. */
    public static double heightToGoal() {
        return GOAL_Z - SHOOTER_EXIT_HEIGHT;
    }

    /** PLACEHOLDER: flywheel RPM for a given horizontal distance (inches) to the goal. */
    public static double rpmForDistance(double distanceIn) {
        return RPM_BASE + RPM_PER_INCH * distanceIn;
    }

    /** PLACEHOLDER: hood servo position for a given horizontal distance (inches) to the goal. */
    public static double hoodForDistance(double distanceIn) {
        return HOOD_BASE + HOOD_PER_INCH * distanceIn;
    }

    /**
     * Turret angle for a goal that is relativeDeg off the robot's heading
     * (positive = left / CCW). 0 deg turret = full left, so left is a smaller turret angle.
     */
    public static double turretForRelativeAngle(double relativeDeg) {
        return TURRET_FORWARD_DEG - relativeDeg;
    }

    // =================================================================
    //  GAMEPAD 2 TRIM
    //    Dpad up / down      = RPM    +/- RPM_STEP
    //    Right / left bumper = hood   +/- HOOD_STEP
    //    Dpad right / left   = turret +/- TURRET_STEP_DEG
    //    Options             = clear all trims
    // =================================================================
    public void handleGamepad(Gamepad gamepad) {
        if (gamepad.dpadUpWasPressed()) rpmTrim += RPM_STEP;
        if (gamepad.dpadDownWasPressed()) rpmTrim -= RPM_STEP;
        if (gamepad.rightBumperWasPressed()) hoodTrim += HOOD_STEP;
        if (gamepad.leftBumperWasPressed()) hoodTrim -= HOOD_STEP;
        if (gamepad.dpadRightWasPressed()) turretTrimDeg += TURRET_STEP_DEG;
        if (gamepad.dpadLeftWasPressed()) turretTrimDeg -= TURRET_STEP_DEG;
        if (gamepad.optionsWasPressed()) {
            rpmTrim = 0;
            hoodTrim = 0;
            turretTrimDeg = 0;
        }
    }

    // =================================================================
    //  UPDATE  (call once per loop)
    // =================================================================
    public void update() {
        double dt = loopTimer.seconds();
        loopTimer.reset();
        if (dt <= 0) return;

        // Re-read the gains every loop so changes made in Panels apply right away.
        leftPid.setGains(FLYWHEEL_KP, FLYWHEEL_KI, FLYWHEEL_KD);
        rightPid.setGains(FLYWHEEL_KP, FLYWHEEL_KI, FLYWHEEL_KD);
        turretPid.setGains(TURRET_KP, TURRET_KI, TURRET_KD);
        turretPid.setOutputLimits(-TURRET_MAX_POWER, TURRET_MAX_POWER);

        // ---- flywheels ----
        double rpmGoal = commandedRpm();
        if (rpmGoal <= 0) {
            leftFlywheel.setPower(0);
            rightFlywheel.setPower(0);
            leftPid.reset();
            rightPid.reset();
        } else {
            double ff = FLYWHEEL_KF * rpmGoal;
            double leftPower = ff + leftPid.update(rpmGoal, getLeftRpm(), dt);
            double rightPower = ff + rightPid.update(rpmGoal, getRightRpm(), dt);
            leftFlywheel.setPower(Range.clip(leftPower, 0, 1));
            rightFlywheel.setPower(Range.clip(rightPower, 0, 1));
        }

        // ---- hood ----
        hood.setPosition(commandedHood());

        // ---- turret ----
        double turretPower = turretPid.update(commandedTurretDeg(), getTurretDegrees(), dt);
        turret.setPower(TURRET_DIRECTION * turretPower);
    }

    // =================================================================
    //  READBACK
    // =================================================================

    /** Target RPM including the driver's trim. */
    public double commandedRpm() {
        return targetRpm <= 0 ? 0 : Range.clip(targetRpm + rpmTrim, 0, FLYWHEEL_MAX_RPM);
    }

    public double commandedHood() {
        return Range.clip(targetHood + hoodTrim, HOOD_MIN, HOOD_MAX);
    }

    public double commandedTurretDeg() {
        return Range.clip(targetTurretDeg + turretTrimDeg, TURRET_MIN_DEG, TURRET_MAX_DEG);
    }

    public double getLeftRpm() {
        return leftFlywheel.getVelocity() / FLYWHEEL_TICKS_PER_REV * 60.0;
    }

    public double getRightRpm() {
        return rightFlywheel.getVelocity() / FLYWHEEL_TICKS_PER_REV * 60.0;
    }

    public double getTurretDegrees() {
        return TURRET_START_DEG + TURRET_DIRECTION * turret.getCurrentPosition() / TURRET_TICKS_PER_DEG;
    }

    public boolean flywheelAtSpeed() {
        double goal = commandedRpm();
        return goal > 0
                && Math.abs(getLeftRpm() - goal) < RPM_TOLERANCE
                && Math.abs(getRightRpm() - goal) < RPM_TOLERANCE;
    }

    public boolean turretOnTarget() {
        return Math.abs(getTurretDegrees() - commandedTurretDeg()) < TURRET_TOLERANCE_DEG;
    }

    /** True when the flywheels are at speed and the turret is pointing where it was told to. */
    public boolean readyToShoot() {
        return flywheelAtSpeed() && turretOnTarget();
    }

    public double getRpmTrim() { return rpmTrim; }
    public double getHoodTrim() { return hoodTrim; }
    public double getTurretTrimDeg() { return turretTrimDeg; }
    public double getLastDistanceToGoal() { return lastDistanceToGoal; }
}
