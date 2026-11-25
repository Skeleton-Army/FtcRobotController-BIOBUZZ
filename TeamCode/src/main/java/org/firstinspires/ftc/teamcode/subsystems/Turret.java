package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.math.Pose;
import com.pedropathing.utils.Angle;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.RobotLog;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.controller.PIDController;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;
import com.seattlesolvers.solverslib.util.MathUtils;
import com.skeletonarmy.marrow.TimerEx;
import java.util.concurrent.TimeUnit;
import java.util.function.Supplier;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.config.interfaces.TurretConfig;
import org.firstinspires.ftc.teamcode.utils.MathUtilities;

public class Turret extends SubsystemBase {
    private final MotorEx turret;
    private final PIDController turretPID;
    private final TurretConfig config;
    private boolean disabled = false;
    private double lastWrappedTarget = 0;
    private boolean hasWrappedTarget = false;
    private double filteredTargetVel = 0;
    private double filteredTargetAccel = 0;
    private double targetVel = 0;
    private double lastTargetAngle = 0;
    private long lastTargetUpdateTime = 0;
    private boolean emergencyStop = false;
    private double voltage;
    private final TimerEx startupTimer;
    private final TimerEx stallTimer;
    private Pose targetPose;
    private final Supplier<Pose> poseSupplier;

    public Turret(final HardwareMap hardwareMap, TurretConfig turretConfig, final Supplier<Pose> poseSupplier) {
        super();
        config = turretConfig;
        turret = new MotorEx(hardwareMap, config.getTurretName(), config.getTurretMotor());
        turret.setRunMode(MotorEx.RunMode.RawPower);
        turret.setZeroPowerBehavior(MotorEx.ZeroPowerBehavior.BRAKE);
        turret.setDistancePerPulse((Math.PI * 2) / (turret.getCPR() * config.getGearRatio()));

        turretPID = new PIDController(config.getTurretKp(), config.getTurretKi(), config.getTurretKd());
        turretPID.setTolerance(config.getTurretPositionTolerance(), config.getTurretVelocityTolerance());

        this.poseSupplier = poseSupplier;
        targetPose = Pose.zero();

        setTargetAngle(0);

        voltage = VoltageGetter.getInstance().getVoltage();

        startupTimer = new TimerEx(TimeUnit.SECONDS);
        stallTimer = new TimerEx(TimeUnit.SECONDS);
    }

    @Override
    public void periodic() {
        startupTimer.start();

        if (!emergencyStop && isTurretDamaged()) {
            emergencyStop = true;
        }

        if (emergencyStop || disabled) {
            turret.stopMotor();
            return;
        }

        setTargetAngle(getTargetAngle());
        updateTurretPID();
    }

    public boolean reachedAngle() {
        return turretPID.atSetPoint();
    }

    public double getAngle(AngleUnit angleUnit) {
        return angleUnit == AngleUnit.DEGREES ? Math.toDegrees(getAngle()) : getAngle();
    }

    public double getTargetAngle(AngleUnit angleUnit) {
        return angleUnit == AngleUnit.DEGREES ? Math.toDegrees(getTargetAngle()) : getTargetAngle();
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

    public void home() {
        //TODO: implement
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

    public void updateVoltage(double voltage) {
        this.voltage = voltage;
    }

    private boolean shouldSkipUpdate(double dt) {
        return dt <= config.getMinUpdateDt() || startupTimer.getElapsed() < config.getStartupSettleTime();
    }

    private double computeRawTargetVel(double currentTargetAngle, double dt) {
        double deltaAngle = Angle.normalizeSigned(currentTargetAngle - lastTargetAngle);
        double rawVel = deltaAngle / dt;
        return MathUtils.clamp(rawVel, -config.getMaxTargetVel(), config.getMaxTargetVel());
    }

    private double computeRawTargetAccel(double rawTargetVel, double dt) {
        double rawAccel = (rawTargetVel - targetVel) / dt;
        return MathUtils.clamp(rawAccel, -config.getMaxTargetAccel(), config.getMaxTargetAccel());
    }

    private void updateFilteredKinematics(double rawTargetVel, double rawTargetAccel) {
        filteredTargetVel =
                MathUtilities.lowPassFilter(rawTargetVel, filteredTargetVel, config.getTurretDerivativeGain());
        filteredTargetAccel = MathUtilities.lowPassFilter(
                rawTargetAccel, filteredTargetAccel, config.getTurretSecondDerivativeGain());
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

    private double wrapToTarget(
            double current, double target, double min, double max, boolean wrap, double lastCommanded) {
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
        return disabled || emergencyStop;
    }

    private double computePID() {
        double measuredPos = turret.getDistance();

        double pid = turretPID.calculate(measuredPos);
        double error = turretPID.getPositionError();

        if (Math.abs(error) > config.getTurretIzone()) {
            turretPID.clearTotalError();
        }

        return pid;
    }

    private double computeFeedforward(double netVel, double netAccel) {
        return (netVel * config.getTurretKv()) + (netAccel * config.getTurretKa());
    }

    private double computeStaticComp(double totalRequest) {
        if (Math.abs(totalRequest) >= config.getTurretMinVoltage()) {
            return (totalRequest > 0) ? config.getTurretKs() : -config.getTurretKs();
        }
        return 0;
    }

    private double clampVoltage(double desiredVoltage) {
        return Math.max(-voltage, Math.min(voltage, desiredVoltage));
    }

    private double calculateTargetAngle(Pose targetPose, Pose robotPose) {
        double heading = robotPose.heading();
        double turretX = robotPose.x()
                + config.getTurretOffsetX() * Math.cos(heading)
                - config.getTurretOffsetY() * Math.sin(heading);
        double turretY = robotPose.y()
                + config.getTurretOffsetX() * Math.sin(heading)
                + config.getTurretOffsetY() * Math.cos(heading);

        double fieldAngle = Math.atan2(targetPose.y() - turretY, targetPose.x() - turretX);
        return Angle.normalizeSigned(fieldAngle - heading);
    }

    private void setTargetAngle(double targetAngleRad) {
        double lastCmd = hasWrappedTarget ? lastWrappedTarget : getAngle();

        double wrapped = wrapToTarget(
                lastCmd, targetAngleRad, config.getTurretMin(), config.getTurretMax(), config.isTurretWrap(), lastCmd);

        lastWrappedTarget = wrapped;
        hasWrappedTarget = true;

        turretPID.setSetPoint(wrapped);
    }

    private boolean isTurretDamaged() {
        boolean motorTryingToMove = Math.abs(turret.getRawPower()) > config.getStallMinPower();

        boolean turretNotMoving = Math.abs(turret.getVelocity()) < config.getStallMinVelocity();

        boolean isStalling = motorTryingToMove && turretNotMoving;

        if (isStalling) {
            stallTimer.start();

            if (stallTimer.getElapsed() > config.getTravelTimeThreshold()) {
                RobotLog.addGlobalWarningMessage("TURRET STALL DETECTED.");
                return true;
            }
        } else {
            stallTimer.pause();
        }

        return false;
    }

    private double getTargetAngle() {
        return calculateTargetAngle(targetPose, poseSupplier.get());
    }

    public void setTargetPose(Pose target) {
        targetPose = target;
    }

}
