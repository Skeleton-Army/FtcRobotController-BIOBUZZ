package org.firstinspires.ftc.teamcode.subsystems;
import static org.firstinspires.ftc.teamcode.config.TurretConfig.*;
import static org.firstinspires.ftc.teamcode.utils.MathUtilities.lowPassFilter;

import com.pedropathing.utils.Angle;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.controller.PIDController;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;
import com.seattlesolvers.solverslib.util.MathUtils;
import com.skeletonarmy.marrow.TimerEx;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.util.concurrent.TimeUnit;

public class Turret extends SubsystemBase {
    private final MotorEx turret;
    private final PIDController turretPID;
    private boolean disabled = false;
    private double lastWrappedTarget = 0;
    private boolean hasWrappedTarget = false;
    private double filteredTargetVel = 0;
    private double filteredTargetAccel = 0;
    private double targetVel = 0;
    private double lastTargetAngle = 0;
    private long lastTargetUpdateTime = 0;
    private boolean emergencyStop = false;
    public double filteredRPM;
    private double voltage = 12;
    public boolean turretDisabled = false;
    private final TimerEx startupTimer;

    public Turret(final HardwareMap hardwareMap) {

        turret = new MotorEx(hardwareMap,TURRET_NAME, TURRET_MOTOR);
        turret.setRunMode(MotorEx.RunMode.RawPower);
        turret.setZeroPowerBehavior(MotorEx.ZeroPowerBehavior.BRAKE);
        turret.setDistancePerPulse((Math.PI * 2) / (turret.getCPR() * GEAR_RATIO));

        turretPID = new PIDController(TURRET_KP, TURRET_KI, TURRET_KD);
        turretPID.setTolerance(TURRET_POSITION_TOLERANCE, TURRET_VELOCITY_TOLERANCE);

        setHorizontalAngle(0);

        startupTimer = new TimerEx(TimeUnit.SECONDS);
        startupTimer.start();
    }

    @Override
    public void periodic() {
        if (disabled || turretDisabled) {
            turret.set(0);
            return;
        }
        updateTurretPID();
    }

    public void setHorizontalAngle(double targetAngleRad) {
        double lastCmd = hasWrappedTarget ? lastWrappedTarget : getAngle();

        double wrapped = wrapToTarget(lastCmd, targetAngleRad, TURRET_MIN, TURRET_MAX, TURRET_WRAP, lastCmd);

        lastWrappedTarget = wrapped;
        hasWrappedTarget = true;

        turretPID.setSetPoint(wrapped);
    }

    public boolean reachedAngle() {
        return turretPID.atSetPoint();
    }

    public double getTurretAngle(AngleUnit angleUnit) {
        return angleUnit == AngleUnit.DEGREES ? Math.toDegrees(getAngle()) : getAngle();
    }

    public void resetTurret() {
        turret.resetEncoder();
        turret.setRunMode(MotorEx.RunMode.RawPower);
    }

    public void disable() {
        disabled = true;
    }

    public void enable() {
        disabled = false;
    }

    public double[] getNetTargetKinematics() {
        long currentTime = System.nanoTime();
        double currentTargetAngle = turretPID.getSetPoint();

        if (lastTargetUpdateTime == 0) {
            recordTargetSample(currentTargetAngle, currentTime);
            return zeroKinematics();
        }

        double dt = (currentTime - lastTargetUpdateTime) / 1e9;

        if (shouldSkipUpdate(dt)) {
            recordTargetSample(currentTargetAngle, currentTime);
            targetVel = 0;
            return zeroKinematics();
        }

        double rawTargetVel = computeRawTargetVel(currentTargetAngle, dt);
        double rawTargetAccel = computeRawTargetAccel(rawTargetVel, dt);

        updateFilteredKinematics(rawTargetVel, rawTargetAccel);

        targetVel = rawTargetVel;
        recordTargetSample(currentTargetAngle, currentTime);

        return computeNetMotion();
    }

    private boolean shouldSkipUpdate(double dt) {
        return dt <= MIN_UPDATE_DT || startupTimer.getElapsed() < STARTUP_SETTLE_TIME;
    }

    private double computeRawTargetVel(double currentTargetAngle, double dt) {
        double deltaAngle = Angle.normalizeSigned(currentTargetAngle - lastTargetAngle);
        double rawVel = deltaAngle / dt;
        return MathUtils.clamp(rawVel, -MAX_TARGET_VEL, MAX_TARGET_VEL);
    }

    private double computeRawTargetAccel(double rawTargetVel, double dt) {
        double rawAccel = (rawTargetVel - targetVel) / dt;
        return MathUtils.clamp(rawAccel, -MAX_TARGET_ACCEL, MAX_TARGET_ACCEL);
    }

    private void updateFilteredKinematics(double rawTargetVel, double rawTargetAccel) {
        filteredTargetVel = lowPassFilter(rawTargetVel, filteredTargetVel, TURRET_DERIVATIVE_GAIN);
        filteredTargetAccel = lowPassFilter(rawTargetAccel, filteredTargetAccel, TURRET_SECOND_DERIVATIVE_GAIN);
    }

    private double[] computeNetMotion() {
        double netVel = filteredTargetVel;
        double netAccel = filteredTargetAccel;
        return new double[] {netVel, netAccel};
    }
    private double[] zeroKinematics() {
        return new double[] {0, 0};
    }

    private void recordTargetSample(double angle, long time) {
        lastTargetAngle = angle;
        lastTargetUpdateTime = time;
    }
    private double wrapToTarget(double current, double target, double min, double max, boolean wrap, double lastCommanded) {
        double delta = Math.atan2(Math.sin(target - current), Math.cos(target - current));
        double closest = current + delta;

        if (!wrap) return MathUtils.clamp(closest, min, max);

        boolean closestInRange = closest >= min && closest <= max;

        double wrapped = (closest < min) ? closest + 2 * Math.PI : closest - 2 * Math.PI;
        boolean wrappedInRange = wrapped >= min && wrapped <= max;

        // Only one side is valid -> unambiguous
        if (closestInRange && !wrappedInRange) return closest;
        if (!closestInRange && wrappedInRange) return wrapped;

        // From here on, only two cases remain: both valid (overlap band) or neither valid (deadzone).
        if (closestInRange) {
            // Overlap band: both sides are technically legal. Stick with
            // whichever is closer to what we last commanded, to avoid flip-flop.
            double distToClosest = Math.abs(closest - lastCommanded);
            double distToWrapped = Math.abs(wrapped - lastCommanded);
            return (distToClosest <= distToWrapped) ? closest : wrapped;
        }

        // Neither side reachable -> true deadzone, clamp to nearest limit
        return (closest < min) ? min : max;
    }

    /** Current turret position in radians. */
    private double getAngle() {
        return turret.getDistance();
    }

    public void updateTurretPID() {
        if (shouldDisableTurret()) {
            turret.set(0);
            return;
        }

        double pid = computePID();

        double[] netKinematics = getNetTargetKinematics();
        double ffBase = computeFeedforward(netKinematics[0], netKinematics[1]);

        double totalRequest = pid + ffBase;
        double staticComp = computeStaticComp(totalRequest);

        double desiredVoltage = clampVoltage(totalRequest + staticComp);

        turret.set(desiredVoltage / voltage);
    }

    private boolean shouldDisableTurret() {
        return disabled || emergencyStop || filteredRPM < MIN_FLYWHEEL_RPM;
    }

    private double computePID() {
        double measuredPos = turret.getDistance();

        double pid = turretPID.calculate(measuredPos);
        double error = turretPID.getPositionError();

        if (Math.abs(error) > TURRET_IZONE) {
            turretPID.clearTotalError();
        }

        return pid;
    }

    private double computeFeedforward(double netVel, double netAccel) {
        return (netVel * TURRET_KV) + (netAccel * TURRET_KA);
    }

    private double computeStaticComp(double totalRequest) {
        if (Math.abs(totalRequest) > TURRET_MIN_VOLTAGE) {
            return (totalRequest > 0) ? TURRET_KS : -TURRET_KS;
        }
        return 0;
    }

    private double clampVoltage(double desiredVoltage) {
        return Math.max(-voltage, Math.min(voltage, desiredVoltage));
    }
}