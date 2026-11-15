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
 * (our own PID on the turret angle, TURRET_MIN_DEG..TURRET_MAX_DEG, no wrapping).
 *
 * The hood is just open or closed. It never changes with the robot's position: the only thing
 * that moves it is the OpMode calling setHoodPosition() (the TeleOp does that on gamepad 1 Y).
 * It starts at HOOD_MIN (closed) when the Shooter is made.
 *
 * Two ways to use it:
 *   - Manual:  setTargetRpm(), setHoodPosition(), setTurretDegrees()
 *   - Auto:    setAlliance() once, then aimAt(robotPose) every loop. It picks the alliance's
 *              goal on the robot's side of the field, sets the flywheels to AIM_RPM and points
 *              the turret at the goal. It does not touch the hood.
 *
 * In both cases call update() once per loop so the flywheel PID runs and the turret target
 * is sent, and handleGamepad(gamepad2)
 * if you want the driver to be able to trim RPM / turret with the dpad.
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
    public static double FLYWHEEL_MAX_RPM = 2809;          // free speed; set from "Shooter 1: Flywheel kF"
    public static double FLYWHEEL_KP = 0.0005;             // power per RPM of error
    public static double FLYWHEEL_KI = 0.0;
    public static double FLYWHEEL_KD = 0.0;
    public static double FLYWHEEL_KF = 0.0003050752;       // feedforward: power per RPM (~1/free-speed RPM to start)
    public static double RPM_TOLERANCE = 75;               // "at speed" when both wheels are within this
    public static boolean RIGHT_FLYWHEEL_REVERSED = true;  // the two wheels usually spin opposite ways

    // =================================================================
    //  HOOD
    // =================================================================
    // HOOD_MIN = closed, HOOD_MAX = open. On this robot the servo number goes DOWN as the
    // hood opens, so HOOD_MIN is the bigger number. setHoodPosition() handles either order.
    public static double HOOD_MAX = 0.45;
    public static double HOOD_MIN = .88;

    // =================================================================
    //  TURRET
    //    The turret is at TURRET_START_DEG (straight ahead) when the OpMode inits, and the
    //    encoder count is 0 there. Turning COUNTERCLOCKWISE (left, seen from above) goes
    //    toward TURRET_MIN_DEG, CLOCKWISE (right) goes toward TURRET_MAX_DEG.
    //
    //    Each end has a degree AND the encoder ticks at that degree. Measure both with
    //    "Shooter 4: Turret Position": jog to each end, read the ticks there, and measure
    //    the angle. Angles in between are worked out in a straight line from those numbers,
    //    so the tick signs (+/-) and motor direction are taken care of automatically.
    //    The turret is never sent past either end.
    // =================================================================
    public static double TURRET_START_DEG = 90;            // where the turret is at init (0 ticks)
    public static double TURRET_FORWARD_DEG = 90;          // turret angle that points along the robot's heading

    public static double TURRET_MIN_DEG = 70;              // PLACEHOLDER: counterclockwise end, degrees
    public static int TURRET_MIN_TICKS = 10;              // PLACEHOLDER: encoder ticks at TURRET_MIN_DEG

    public static double TURRET_MAX_DEG = 120;             // PLACEHOLDER: clockwise end, degrees
    public static int TURRET_MAX_TICKS = -15;               // PLACEHOLDER: encoder ticks at TURRET_MAX_DEG

    // Turret PID, on the angle in DEGREES (so the gains don't change if the gearing does).
    // Tune with "Shooter 5: Turret PID". Output = motor power, capped at +/- TURRET_MAX_POWER.
    public static double TURRET_KP = 0.015;                 // power per degree of error (PLACEHOLDER)
    public static double TURRET_KI = 0.0;                  // power per degree-second of error
    public static double TURRET_KD = 0.001;               // power per degree/second (damping)
    public static double TURRET_KS = 0.0;                  // extra power to overcome friction, added outside TURRET_TOLERANCE_DEG
    public static double TURRET_I_LIMIT = 20;              // cap on the integral (degree-seconds) so it can't wind up
    public static double TURRET_MAX_POWER = 0.6;           // max motor power the PID can give
    public static double TURRET_TOLERANCE_DEG = 3.0;       // "on target" within this; KS turns off inside it

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

    /** RPM aimAt() always uses for now, whatever the distance. Editable in Panels. */
    public static double AIM_RPM = 2400;

    // PLACEHOLDER formula, linear in horizontal distance (aimAt() doesn't use it right now).
    //   rpm = RPM_BASE + RPM_PER_INCH * distance
    public static double RPM_BASE = 2500;
    public static double RPM_PER_INCH = 10;

    // =================================================================
    //  GAMEPAD 2 TRIM STEPS
    // =================================================================
    public static double RPM_STEP = 50;
    public static double TURRET_STEP_DEG = 2;

    // ---------------- hardware ----------------
    private final DcMotorEx leftFlywheel;
    private final DcMotorEx rightFlywheel;
    private final Servo hood;
    private final DcMotorEx turret;   // null when made with useTurret = false

    private final PidController leftPid = new PidController(FLYWHEEL_KP, FLYWHEEL_KI, FLYWHEEL_KD);
    private final PidController rightPid = new PidController(FLYWHEEL_KP, FLYWHEEL_KI, FLYWHEEL_KD);
    private final PidController turretPid = new PidController(TURRET_KP, TURRET_KI, TURRET_KD);
    private double turretPower = 0;   // last power sent to the turret, for telemetry
    private final ElapsedTime loopTimer = new ElapsedTime();

    // ---------------- targets (before trim) ----------------
    private double targetRpm = 0;
    private double targetHood = HOOD_MIN;
    private double targetTurretDeg = TURRET_START_DEG;

    // ---------------- driver trims (gamepad 2) ----------------
    private double rpmTrim = 0;
    private double turretTrimDeg = 0;

    // ---------------- aiming ----------------
    private Alliance alliance = Alliance.BLUE;
    private Goal activeGoal = BLUE_FAR;
    private double lastDistanceToGoal = 0;

    /** Shooter with the turret. */
    public Shooter(HardwareMap hardwareMap) {
        this(hardwareMap, true);
    }

    /**
     * useTurret = false leaves the turret out completely: it is never looked up in the
     * hardware map (so it doesn't need to be in the robot config) and never commanded.
     * Turret setters and trims are then ignored, and turretOnTarget() is always true.
     */
    public Shooter(HardwareMap hardwareMap, boolean useTurret) {
        leftFlywheel = hardwareMap.get(DcMotorEx.class, LEFT_FLYWHEEL_NAME);
        rightFlywheel = hardwareMap.get(DcMotorEx.class, RIGHT_FLYWHEEL_NAME);
        hood = hardwareMap.get(Servo.class, HOOD_NAME);
        turret = useTurret ? hardwareMap.get(DcMotorEx.class, TURRET_NAME) : null;

        // We run our own PID, so the SDK's built-in velocity control is turned off.
        leftFlywheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFlywheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftFlywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightFlywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightFlywheel.setDirection(RIGHT_FLYWHEEL_REVERSED
                ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);

        // The turret is wherever TURRET_START_DEG says it is at init; the count resets there.
        // Our PID does the position control, so the motor runs on raw power (the encoder still counts).
        if (turret != null) {
            turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        leftPid.setOutputLimits(-1, 1);
        rightPid.setOutputLimits(-1, 1);

        hood.setPosition(HOOD_MIN);
        loopTimer.reset();
    }

    // =================================================================
    //  MANUAL SETTERS
    // =================================================================

    /** Flywheel speed in RPM. 0 turns the flywheels off. */
    public void setTargetRpm(double rpm) {
        targetRpm = Range.clip(rpm, 0, FLYWHEEL_MAX_RPM);
    }

    /** Hood servo position, kept between HOOD_MIN and HOOD_MAX (whichever order they are in). */
    public void setHoodPosition(double position) {
        targetHood = Range.clip(position, Math.min(HOOD_MIN, HOOD_MAX), Math.max(HOOD_MIN, HOOD_MAX));
    }

    /** Turret angle in degrees, clipped to TURRET_MIN_DEG..TURRET_MAX_DEG. */
    public void setTurretDegrees(double degrees) {
        targetTurretDeg = Range.clip(degrees, TURRET_MIN_DEG, TURRET_MAX_DEG);
    }

    /** Flywheels off, turret to the forward position. The hood is left where it is. */
    public void stop() {
        setTargetRpm(0);
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
     * Sets the flywheels to AIM_RPM and points the turret at the goal on the robot's side of
     * the field, from the robot's field pose. Leaves the hood alone. Call every loop while
     * lining up, then update().
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

        // Fixed RPM for now. Switch to rpmForDistance(lastDistanceToGoal) once the shot table is tuned.
        setTargetRpm(AIM_RPM);

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
    //    Dpad right / left   = turret +/- TURRET_STEP_DEG
    //    Options             = clear all trims
    //    (No hood trim: the hood only moves when the OpMode calls setHoodPosition.)
    // =================================================================
    public void handleGamepad(Gamepad gamepad) {
        if (gamepad.dpadUpWasPressed()) rpmTrim += RPM_STEP;
        if (gamepad.dpadDownWasPressed()) rpmTrim -= RPM_STEP;
        if (turret != null) {
            if (gamepad.dpadRightWasPressed()) turretTrimDeg += TURRET_STEP_DEG;
            if (gamepad.dpadLeftWasPressed()) turretTrimDeg -= TURRET_STEP_DEG;
        }
        if (gamepad.optionsWasPressed()) {
            rpmTrim = 0;
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
        if (turret != null) {
            turretPower = turretPidPower(commandedTurretDeg(), getTurretDegrees(), dt);
            turret.setPower(turretPower);
        }
    }

    /**
     * One step of the turret PID: motor power that moves the turret from currentDeg toward
     * targetDeg. Gains are re-read every call so Panels changes apply straight away.
     */
    private double turretPidPower(double targetDeg, double currentDeg, double dt) {
        turretPid.setGains(TURRET_KP, TURRET_KI, TURRET_KD);
        turretPid.setIntegralLimits(-TURRET_I_LIMIT, TURRET_I_LIMIT);
        double maxPower = Range.clip(TURRET_MAX_POWER, 0, 1);

        double error = targetDeg - currentDeg;
        double power = turretPid.updateFromError(error, dt);
        if (Math.abs(error) > TURRET_TOLERANCE_DEG) power += Math.signum(error) * TURRET_KS;
        power = Range.clip(power, -maxPower, maxPower);

        // The PID works in degrees: positive = toward TURRET_MAX_DEG. Positive motor power
        // turns toward bigger tick counts, so flip it if TURRET_MAX_TICKS is the smaller count.
        return TURRET_MAX_TICKS >= TURRET_MIN_TICKS ? power : -power;
    }

    /**
     * Encoder count that puts the turret at the given angle. Inverse of turretDegreesForTicks().
     * Counterclockwise of start it scales between (TURRET_START_DEG, 0) and
     * (TURRET_MIN_DEG, TURRET_MIN_TICKS); clockwise of start between (TURRET_START_DEG, 0)
     * and (TURRET_MAX_DEG, TURRET_MAX_TICKS).
     */
    public static int turretTicksForDegrees(double degrees) {
        double offset = degrees - TURRET_START_DEG;
        if (offset < 0) {
            double span = TURRET_MIN_DEG - TURRET_START_DEG;
            return span == 0 ? 0 : (int) Math.round(offset / span * TURRET_MIN_TICKS);
        } else {
            double span = TURRET_MAX_DEG - TURRET_START_DEG;
            return span == 0 ? 0 : (int) Math.round(offset / span * TURRET_MAX_TICKS);
        }
    }

    /** Turret angle for an encoder count. Inverse of turretTicksForDegrees(). */
    public static double turretDegreesForTicks(int ticks) {
        if (ticks == 0) return TURRET_START_DEG;
        // Which side of start is this? The side whose end has ticks of the same sign.
        boolean ccwSide = TURRET_MIN_TICKS != 0 && Math.signum(ticks) == Math.signum(TURRET_MIN_TICKS);
        if (ccwSide) {
            return TURRET_START_DEG + (double) ticks / TURRET_MIN_TICKS * (TURRET_MIN_DEG - TURRET_START_DEG);
        }
        if (TURRET_MAX_TICKS == 0) return TURRET_START_DEG;
        return TURRET_START_DEG + (double) ticks / TURRET_MAX_TICKS * (TURRET_MAX_DEG - TURRET_START_DEG);
    }

    // =================================================================
    //  READBACK
    // =================================================================

    /** Target RPM including the driver's trim. */
    public double commandedRpm() {
        return targetRpm <= 0 ? 0 : Range.clip(targetRpm + rpmTrim, 0, FLYWHEEL_MAX_RPM);
    }

    public double commandedHood() {
        return targetHood;   // already kept inside HOOD_MIN / HOOD_MAX by setHoodPosition
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

    /** Turret angle from the encoder. Without a turret this is just where it is told to be. */
    public double getTurretDegrees() {
        if (turret == null) return commandedTurretDeg();
        return turretDegreesForTicks(turret.getCurrentPosition());
    }

    /** Raw turret encoder count since init. 0 without a turret. */
    public int getTurretTicks() {
        return turret == null ? 0 : turret.getCurrentPosition();
    }

    /** Last power the turret PID sent to the motor (-TURRET_MAX_POWER..TURRET_MAX_POWER). 0 without a turret. */
    public double getTurretPower() {
        return turretPower;
    }

    /** Turret PID error in degrees (target - current). */
    public double getTurretErrorDeg() {
        return commandedTurretDeg() - getTurretDegrees();
    }

    /** False when this Shooter was made with useTurret = false. */
    public boolean hasTurret() {
        return turret != null;
    }

    public boolean flywheelAtSpeed() {
        double goal = commandedRpm();
        return goal > 0
                && Math.abs(getLeftRpm() - goal) < RPM_TOLERANCE
                && Math.abs(getRightRpm() - goal) < RPM_TOLERANCE;
    }

    public boolean turretOnTarget() {
        if (turret == null) return true;
        return Math.abs(getTurretDegrees() - commandedTurretDeg()) < TURRET_TOLERANCE_DEG;
    }

    /** True when the flywheels are at speed and the turret is pointing where it was told to. */
    public boolean readyToShoot() {
        return flywheelAtSpeed() && turretOnTarget();
    }

    public double getRpmTrim() { return rpmTrim; }
    public double getTurretTrimDeg() { return turretTrimDeg; }
    public double getLastDistanceToGoal() { return lastDistanceToGoal; }
}
