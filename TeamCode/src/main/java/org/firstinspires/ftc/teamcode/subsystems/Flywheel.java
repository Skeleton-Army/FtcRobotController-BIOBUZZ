package org.firstinspires.ftc.teamcode.subsystems;

import androidx.core.math.MathUtils;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.RobotLog;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.controller.PIDController;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;
import com.skeletonarmy.marrow.TimerEx;
import java.util.concurrent.TimeUnit;
import org.firstinspires.ftc.teamcode.config.interfaces.FlywheelConfig;
import org.firstinspires.ftc.teamcode.utils.MathUtilities;
import org.firstinspires.ftc.teamcode.utils.MovingAverage;

public class Flywheel extends SubsystemBase {
    private final MotorEx flywheel;
    private final PIDController flywheelPID;
    private final FlywheelConfig config;
    private final double NOMINAL_VOLTAGE = 12;
    private double voltage = NOMINAL_VOLTAGE;
    private double targetTPS;
    private double lastSpeedFlywheel = 0;
    private long lastLoopTime = 0;
    private final TimerEx rampTimer;
    private final TimerEx stallTimer;
    private static final double SECONDS_PER_MINUTE = 60.0;
    private static final double NANOS_PER_SECOND = 1.0e9;
    private static final int RPM_WINDOW_SIZE = 5;
    private final MovingAverage filteredRPM = new MovingAverage(RPM_WINDOW_SIZE);
    private double lastFilteredRPM = 0;
    private long lastFilteredRPMTime = 0;
    private double filteredRPMAccel = 0; // RPM/s, derived from filteredRPM
    private boolean disabled = false;
    private boolean emergencyStop = false;

    public Flywheel(final HardwareMap hardwareMap, FlywheelConfig flywheelConfig) {
        super();
        this.config = flywheelConfig;
        flywheel = new MotorEx(
                hardwareMap,
                config.getFlywheelName(),
                config.getFlywheelMotor().getCPR() * config.getFlywheelGearRatio(),
                config.getFlywheelMotor().getRPM() / config.getFlywheelGearRatio());
        flywheel.setInverted(config.isFlywheelInverted());

        flywheel.setVeloCoefficients(config.getFlywheelKp(), config.getFlywheelKi(), config.getFlywheelKd());
        flywheel.setFeedforwardCoefficients(config.getFlywheelKs(), config.getFlywheelKv(), config.getFlywheelKa());
        flywheel.setRunMode(MotorEx.RunMode.RawPower);

        flywheelPID = new PIDController(config.getFlywheelKp(), config.getFlywheelKi(), config.getFlywheelKd());

        rampTimer = new TimerEx(TimeUnit.SECONDS);

        stallTimer = new TimerEx(TimeUnit.SECONDS);
    }

    @Override
    public void periodic() {
        rampTimer.start();

        filteredRPM.update(getUnfilteredRPM());

        updateFilteredRPMAccel();

        if (isFlywheelDamaged() && !emergencyStop) {
            flywheel.stopMotor();
            emergencyStop = true;
            return;
        }

        updateFlywheelPIDFiltered();
    }

    public boolean isFlywheelDamaged() {
        double currentRPM = Math.abs(getRPM());
        double targetRPM = Math.abs(getTargetRPM());

        // 1. Encoder Direction Check
        if ((getRPM() < -config.getWrongDirectionRpmThreshold() && targetRPM > 0)
                || (getRPM() > config.getWrongDirectionRpmThreshold() && targetRPM < 0)) {
            RobotLog.addGlobalWarningMessage("FLYWHEEL IS SPINNING IN THE WRONG DIRECTION.");
            return true;
        }

        // 2. Conflict/Stall Check
        boolean isStalling =
                targetRPM > config.getStallTargetRpmThreshold() && currentRPM < config.getStallActualRpmThreshold();

        if (isStalling) {
            stallTimer.start();

            if (stallTimer.getElapsed() > config.getStallTimeout()) {
                RobotLog.addGlobalWarningMessage(
                        "FLYWHEEL STALL DETECTED. ONE OF THE FLYWHEEL MOTORS IS PROBABLY REVERSED.");
                return true;
            }
        } else {
            stallTimer.pause();
        }

        return false;
    }

    public double getFilteredRPMAccel() {
        return filteredRPMAccel;
    }

    public double getTargetRPM() {
        return (targetTPS * SECONDS_PER_MINUTE) / flywheel.getCPR();
    }

    public void setRPM(double rpm) {
        rpm = MathUtils.clamp(rpm, 0, flywheel.getMaxRPM());
        this.targetTPS = (rpm * flywheel.getCPR()) / SECONDS_PER_MINUTE;
    }

    public void updateVoltage(double voltage) {
        this.voltage = voltage;
    }

    public void disable() {
        disabled = true;
        flywheel.stopMotor();
    }

    public void enable() {
        disabled = false;
    }

    public double getRPM() {
        return filteredRPM.get();
    }

    /**
     * Derives flywheel angular acceleration from the already-averaged filteredRPM signal.
     * Since filteredRPM is a moving average, its derivative is far less noisy than
     * differentiating the raw encoder velocity directly.
     */
    private void updateFilteredRPMAccel() {
        long currentTime = System.nanoTime();
        double currentRPM = getRPM();

        if (lastFilteredRPMTime == 0) {
            lastFilteredRPMTime = currentTime;
            lastFilteredRPM = currentRPM;
        }

        double dt = (currentTime - lastFilteredRPMTime) / NANOS_PER_SECOND;
        if (dt < config.getMinDtSeconds()) dt = config.getMinDtSeconds();

        double rawAccel = (currentRPM - lastFilteredRPM) / dt; // RPM/s
        filteredRPMAccel =
                MathUtilities.lowPassFilter(rawAccel, filteredRPMAccel, config.getFlywheelAccelFilterAlpha());

        lastFilteredRPM = currentRPM;
        lastFilteredRPMTime = currentTime;
    }

    private void updateFlywheelPIDFiltered() {
        if (disabled || emergencyStop) {
            flywheel.set(0);
            rampTimer.restart();
            rampTimer.pause();
            lastSpeedFlywheel = 0;
            lastLoopTime = System.nanoTime();
            return;
        }

        // 1. Calculate real Delta Time (dt)
        long currentTime = System.nanoTime();
        if (lastLoopTime == 0) lastLoopTime = currentTime;
        double dt = (currentTime - lastLoopTime) / NANOS_PER_SECOND;
        lastLoopTime = currentTime;
        if (dt < config.getMinDtSeconds()) dt = config.getMinDtSeconds();

        if (!rampTimer.isOn()) rampTimer.resume();
        double rampMultiplier = Math.min(rampTimer.getElapsed() / config.getInitialRampDuration(), 1.0);

        // 'speed' is our Target Velocity
        double speed = targetTPS * rampMultiplier;

        double tpsPerRPM = flywheel.getCPR() / SECONDS_PER_MINUTE;
        double processVariable = getRPM() * tpsPerRPM;

        flywheelPID.setPID(config.getFlywheelKp(), config.getFlywheelKi(), config.getFlywheelKd());

        // 3. Calculate Acceleration for Feedforward
        double targetAcceleration = (speed - lastSpeedFlywheel) / dt;
        double currentKA = config.getFlywheelKa();

        // PID and FF now output volts directly
        double pid = flywheelPID.calculate(processVariable, speed);

        // Feedforward calculates based on TARGET (speed), not measurement
        double ff = (config.getFlywheelKs() * Math.signum(speed))
                + (config.getFlywheelKv() * speed)
                + (currentKA * targetAcceleration);

        lastSpeedFlywheel = speed;

        double desiredVoltage = pid + ff;
        desiredVoltage = Math.max(-voltage, Math.min(voltage, desiredVoltage));

        flywheel.set(desiredVoltage / voltage);
    }

    private double getUnfilteredRPM() {
        double motorTPS = flywheel.getCorrectedVelocity();
        return (motorTPS * SECONDS_PER_MINUTE) / flywheel.getCPR();
    }
}
