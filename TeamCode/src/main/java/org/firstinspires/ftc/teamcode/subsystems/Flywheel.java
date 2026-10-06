package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.config.FlywheelConfig.*;

import androidx.core.math.MathUtils;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.RobotLog;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.controller.PIDController;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;
import com.seattlesolvers.solverslib.hardware.motors.MotorGroup;
import com.skeletonarmy.marrow.TimerEx;

import java.util.concurrent.TimeUnit;

public class Flywheel extends SubsystemBase {
    private final MotorEx flywheel1;
    private final MotorEx flywheel2;
    private final MotorGroup flywheel;
    private final PIDController flywheelPID;

    private final VoltageSensor voltageSensor;
    private double voltage = 12;
    private boolean voltageExternallySupplied = false;

    private double targetTPS;
    private double lastSpeedFlywheel = 0;
    private long lastLoopTime = 0;
    private boolean isBraking = false;

    private final TimerEx rampTimer;
    private final TimerEx stallTimer;

    private final double[] rpmBuffer = new double[RPM_WINDOW_SIZE];
    private int bufferIndex = 0;
    private double runningRpmSum = 0;
    public double filteredRPM;

    private double lastFilteredRPM = 0;
    private long lastFilteredRPMTime = 0;
    private double filteredRPMAccel = 0; // RPM/s, derived from filteredRPM

    private double lastShotRPM;

    public boolean disabled = false;
    private boolean emergencyStop = false;

    public Flywheel(final HardwareMap hardwareMap) {
        flywheel1 = new MotorEx(hardwareMap, FLYWHEEL1_NAME,
                FLYWHEEL_MOTOR.getCPR() * FLYWHEEL_GEAR_RATIO,
                FLYWHEEL_MOTOR.getRPM() / FLYWHEEL_GEAR_RATIO);
        flywheel1.setInverted(FLYWHEEL1_INVERTED);

        flywheel2 = new MotorEx(hardwareMap, FLYWHEEL2_NAME,
                FLYWHEEL_MOTOR.getCPR() * FLYWHEEL_GEAR_RATIO,
                FLYWHEEL_MOTOR.getRPM() / FLYWHEEL_GEAR_RATIO);
        flywheel2.setInverted(FLYWHEEL2_INVERTED);

        flywheel = new MotorGroup(flywheel1, flywheel2);
        flywheel.setVeloCoefficients(FLYWHEEL_KP, FLYWHEEL_KI, FLYWHEEL_KD);
        flywheel.setFeedforwardCoefficients(FLYWHEEL_KS, FLYWHEEL_KV, FLYWHEEL_KA);
        flywheel.setRunMode(MotorEx.RunMode.RawPower);


        flywheelPID = new PIDController(FLYWHEEL_KP, FLYWHEEL_KI, FLYWHEEL_KD);

        rampTimer = new TimerEx(TimeUnit.SECONDS);
        rampTimer.start();
        stallTimer = new TimerEx(TimeUnit.SECONDS);

        voltageSensor = hardwareMap.voltageSensor.iterator().next();
    }

    @Override
    public void periodic() {
        if (!voltageExternallySupplied) {
            this.voltage = voltageSensor.getVoltage();
        }

        filteredRPM = getFilteredRPM(getRPM());
        updateFilteredRPMAccel();

        if (isFlywheelDamaged() && !emergencyStop) {
            flywheel.stopMotor();
            emergencyStop = true;
            return;
        }

        updateFlywheelPIDFiltered();

        voltageExternallySupplied = false;
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
        double dt = (currentTime - lastLoopTime) / 1.0e9;
        lastLoopTime = currentTime;
        if (dt < 0.001) dt = 0.001;

        if (!rampTimer.isOn()) rampTimer.resume();
        double rampMultiplier = Math.min(rampTimer.getElapsed() / INITIAL_RAMP_DURATION, 1.0);

        // 'speed' is our Target Velocity
        double speed = targetTPS * rampMultiplier;

        double tpsPerRPM = flywheel.getCPR() / 60.0;
        double processVariable = filteredRPM * tpsPerRPM;

        // --- ASYMMETRIC P-GAIN LOGIC ---
        double error = speed - processVariable;

        if (!isBraking && error < BRAKE_ENTRY_THRESHOLD * tpsPerRPM) {
            isBraking = true;
        } else if (isBraking && error > BRAKE_EXIT_THRESHOLD * tpsPerRPM) {
            isBraking = false;
        }

        if (isBraking) {
            flywheelPID.setPID(FLYWHEEL_KP_DOWN, FLYWHEEL_KI, FLYWHEEL_KD);
        } else {
            flywheelPID.setPID(FLYWHEEL_KP, FLYWHEEL_KI, FLYWHEEL_KD);
        }

        // 3. Calculate Acceleration for Feedforward
        double targetAcceleration = (speed - lastSpeedFlywheel) / dt;
        double currentKA = (targetAcceleration >= 0) ? FLYWHEEL_KA : FLYWHEEL_KA_DOWN;

        // PID and FF now output volts directly
        double pid = flywheelPID.calculate(processVariable, speed);

        // Feedforward calculates based on TARGET (speed), not measurement
        double ff = (FLYWHEEL_KS * Math.signum(speed)) +
                (FLYWHEEL_KV * speed) +
                (currentKA * targetAcceleration);

        lastSpeedFlywheel = speed;

        double desiredVoltage = pid + ff;
        desiredVoltage = Math.max(-voltage, Math.min(voltage, desiredVoltage));

        flywheel.set(desiredVoltage / voltage);
    }

    public boolean isFlywheelDamaged() {
        double currentRPM = Math.abs(getRPM());
        double targetRPM = Math.abs(getTargetRPM());

        // 1. Encoder Direction Check
        if ((getRPM() < -500 && targetRPM > 0) || (getRPM() > 500 && targetRPM < 0)) {
            RobotLog.addGlobalWarningMessage("FLYWHEEL IS SPINNING IN THE WRONG DIRECTION.");
            return true;
        }

        // 2. Conflict/Stall Check
        boolean isStalling = targetRPM > 1000 && currentRPM < 200;

        if (isStalling) {
            stallTimer.start();
            stallTimer.resume();

            if (stallTimer.getElapsed() > STALL_TIMEOUT) {
                RobotLog.addGlobalWarningMessage("FLYWHEEL STALL DETECTED. ONE OF THE FLYWHEEL MOTORS IS PROBABLY REVERSED.");
                return true;
            }
        } else {
            stallTimer.restart();
            stallTimer.pause();
        }

        return false;
    }

    public double getRPM() {
        double motorTPS = flywheel.getVelocity();
        return (motorTPS * 60.0) / flywheel.getCPR();
    }

    /**
     * Derives flywheel angular acceleration from the already-averaged filteredRPM signal.
     * Since filteredRPM is a moving average, its derivative is far less noisy than
     * differentiating the raw encoder velocity directly.
     */
    private double updateFilteredRPMAccel() {
        long currentTime = System.nanoTime();

        if (lastFilteredRPMTime == 0) {
            lastFilteredRPMTime = currentTime;
            lastFilteredRPM = filteredRPM;
            return 0;
        }

        double dt = (currentTime - lastFilteredRPMTime) / 1e9;
        if (dt < 0.001) dt = 0.001;

        double rawAccel = (filteredRPM - lastFilteredRPM) / dt; // RPM/s
        filteredRPMAccel = lowPassFilter(rawAccel, filteredRPMAccel, FLYWHEEL_ACCEL_FILTER_ALPHA);

        lastFilteredRPM = filteredRPM;
        lastFilteredRPMTime = currentTime;

        return filteredRPMAccel;
    }

    public double getFilteredRPMAccel() {
        return filteredRPMAccel;
    }

    public double getRPMCorrectedTiming() {
        double motorTPS = flywheel.getVelocity();
        if (!Double.isNaN(flywheel1.getAcceleration())) {
            motorTPS += flywheel1.getAcceleration() * FLYWHEEL_SHOOTING_DIFFERENCE;
            return (motorTPS * 60.0) / flywheel.getCPR();
        }

        return (motorTPS * 60.0) / flywheel.getCPR();
    }

    public double getTargetRPM() {
        return (targetTPS * 60.0) / flywheel.getCPR();
    }

    private double getFilteredRPM(double currentRPM) {
        runningRpmSum -= rpmBuffer[bufferIndex];
        rpmBuffer[bufferIndex] = currentRPM;
        runningRpmSum += currentRPM;
        bufferIndex = (bufferIndex + 1) % RPM_WINDOW_SIZE;
        return runningRpmSum / RPM_WINDOW_SIZE;
    }

    public void setRPM(double rpm) {
        rpm = MathUtils.clamp(rpm, 0, flywheel.getMaxRPM());
        this.targetTPS = (rpm * flywheel.getCPR()) / 60.0;
    }

    public void updateVoltage(double voltage) {
        this.voltage = voltage;
        voltageExternallySupplied = true;
    }

    public boolean justShot() {
        if ((Math.abs(getTargetRPM() - getRPM()) > RPM_REACHED_THRESHOLD) && (Math.abs(lastShotRPM - getRPM()) > RPM_REACHED_THRESHOLD)) {
            return true;
        }

        lastShotRPM = getRPM();
        return false;
    }

    public void disable() {
        disabled = true;
        flywheel.stopMotor();
    }

    public void enable() {
        disabled = false;
    }

    private static double lowPassFilter(double newVal, double oldVal, double gain) {
        return (gain * newVal) + ((1.0 - gain) * oldVal);
    }
}