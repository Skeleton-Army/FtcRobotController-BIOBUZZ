package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.config.FlywheelConfig.*;

import androidx.core.math.MathUtils;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.RobotLog;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.controller.PIDController;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;
import com.skeletonarmy.marrow.TimerEx;
import java.util.concurrent.TimeUnit;
import org.firstinspires.ftc.teamcode.utils.MathUtilities;

public class Flywheel extends SubsystemBase {
    private final MotorEx flywheel;
    private final PIDController flywheelPID;
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
    private final double[] rpmBuffer = new double[RPM_WINDOW_SIZE];
    private int bufferIndex = 0;
    private double runningRpmSum = 0;
    public double filteredRPM;
    private double lastFilteredRPM = 0;
    private long lastFilteredRPMTime = 0;
    private double filteredRPMAccel = 0; // RPM/s, derived from filteredRPM
    public boolean disabled = false;
    private boolean emergencyStop = false;

    public Flywheel(final HardwareMap hardwareMap) {
        super();
        flywheel = new MotorEx(
                hardwareMap,
                FLYWHEEL_NAME,
                FLYWHEEL_MOTOR.getCPR() * FLYWHEEL_GEAR_RATIO,
                FLYWHEEL_MOTOR.getRPM() / FLYWHEEL_GEAR_RATIO);
        flywheel.setInverted(FLYWHEEL_INVERTED);

        flywheel.setVeloCoefficients(FLYWHEEL_KP, FLYWHEEL_KI, FLYWHEEL_KD);
        flywheel.setFeedforwardCoefficients(FLYWHEEL_KS, FLYWHEEL_KV, FLYWHEEL_KA);
        flywheel.setRunMode(MotorEx.RunMode.RawPower);

        flywheelPID = new PIDController(FLYWHEEL_KP, FLYWHEEL_KI, FLYWHEEL_KD);

        rampTimer = new TimerEx(TimeUnit.SECONDS);

        stallTimer = new TimerEx(TimeUnit.SECONDS);
    }

    @Override
    public void periodic() {
        rampTimer.start();

        filteredRPM = getFilteredRpm(getRPM());
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
        if ((getRPM() < -WRONG_DIRECTION_RPM_THRESHOLD && targetRPM > 0)
                || (getRPM() > WRONG_DIRECTION_RPM_THRESHOLD && targetRPM < 0)) {
            RobotLog.addGlobalWarningMessage("FLYWHEEL IS SPINNING IN THE WRONG DIRECTION.");
            return true;
        }

        // 2. Conflict/Stall Check
        boolean isStalling = targetRPM > STALL_TARGET_RPM_THRESHOLD && currentRPM < STALL_ACTUAL_RPM_THRESHOLD;

        if (isStalling) {
            stallTimer.start();

            if (stallTimer.getElapsed() > STALL_TIMEOUT) {
                RobotLog.addGlobalWarningMessage(
                        "FLYWHEEL STALL DETECTED. ONE OF THE FLYWHEEL MOTORS IS PROBABLY REVERSED.");
                return true;
            }
        } else {
            stallTimer.pause();
        }

        return false;
    }

    public double getRPM() {
        double motorTPS = flywheel.getCorrectedVelocity();
        return (motorTPS * SECONDS_PER_MINUTE) / flywheel.getCPR();
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

    public double getFilteredRpm(double currentRPM) {
        runningRpmSum -= rpmBuffer[bufferIndex];
        rpmBuffer[bufferIndex] = currentRPM;
        runningRpmSum += currentRPM;
        bufferIndex = (bufferIndex + 1) % RPM_WINDOW_SIZE;
        return runningRpmSum / RPM_WINDOW_SIZE;
    }

    /**
     * Derives flywheel angular acceleration from the already-averaged filteredRPM signal.
     * Since filteredRPM is a moving average, its derivative is far less noisy than
     * differentiating the raw encoder velocity directly.
     */
    private void updateFilteredRPMAccel() {
        long currentTime = System.nanoTime();

        if (lastFilteredRPMTime == 0) {
            lastFilteredRPMTime = currentTime;
            lastFilteredRPM = filteredRPM;
        }

        double dt = (currentTime - lastFilteredRPMTime) / NANOS_PER_SECOND;
        if (dt < MIN_DT_SECONDS) dt = MIN_DT_SECONDS;

        double rawAccel = (filteredRPM - lastFilteredRPM) / dt; // RPM/s
        filteredRPMAccel = MathUtilities.lowPassFilter(rawAccel, filteredRPMAccel, FLYWHEEL_ACCEL_FILTER_ALPHA);

        lastFilteredRPM = filteredRPM;
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
        if (dt < MIN_DT_SECONDS) dt = MIN_DT_SECONDS;

        if (!rampTimer.isOn()) rampTimer.resume();
        double rampMultiplier = Math.min(rampTimer.getElapsed() / INITIAL_RAMP_DURATION, 1.0);

        // 'speed' is our Target Velocity
        double speed = targetTPS * rampMultiplier;

        double tpsPerRPM = flywheel.getCPR() / SECONDS_PER_MINUTE;
        double processVariable = filteredRPM * tpsPerRPM;

        flywheelPID.setPID(FLYWHEEL_KP, FLYWHEEL_KI, FLYWHEEL_KD);

        // 3. Calculate Acceleration for Feedforward
        double targetAcceleration = (speed - lastSpeedFlywheel) / dt;
        double currentKA = FLYWHEEL_KA;

        // PID and FF now output volts directly
        double pid = flywheelPID.calculate(processVariable, speed);

        // Feedforward calculates based on TARGET (speed), not measurement
        double ff = (FLYWHEEL_KS * Math.signum(speed)) + (FLYWHEEL_KV * speed) + (currentKA * targetAcceleration);

        lastSpeedFlywheel = speed;

        double desiredVoltage = pid + ff;
        desiredVoltage = Math.max(-voltage, Math.min(voltage, desiredVoltage));

        flywheel.set(desiredVoltage / voltage);
    }
}
