package org.firstinspires.ftc.teamcode.subsystems;

import com.seattlesolvers.solverslib.command.SubsystemBase;
import org.firstinspires.ftc.teamcode.config.PollenShooterConfig;
import org.firstinspires.ftc.teamcode.config.ShooterConfig;

/**
 * Subsystem placeholder for controlling the flywheel launcher velocity.
 * Responsible for accelerating game elements to desired spin rates and speeds.
 */
public class Flywheel extends SubsystemBase {

    private String motor1Name;
    private String motor2Name;
    private double kp;
    private double ki;
    private double kd;
    private double kf;
    private double ticksPerRev;
    private double maxRPM;

    private double targetRPM = 0.0;
    private double currentRPM = 0.0;
    private double toleranceRPM = 50.0;

    /**
     * Constructs a Flywheel subsystem configured using constants from a {@link ShooterConfig}.
     * Throws an {@link IllegalArgumentException} if config is null.
     *
     * @param config Configuration containing flywheel parameters
     */
    public Flywheel(ShooterConfig config) {
        setName("FlywheelSubsystem");
        if (config == null) {
            throw new IllegalArgumentException("ShooterConfig cannot be null");
        }
        this.motor1Name = config.getFlywheelMotor1Name();
        this.motor2Name = config.getFlywheelMotor2Name();
        this.kp = config.getFlywheelKp();
        this.ki = config.getFlywheelKi();
        this.kd = config.getFlywheelKd();
        this.kf = config.getFlywheelKf();
        this.ticksPerRev = config.getFlywheelTicksPerRev();
        this.maxRPM = config.getMaxFlywheelRPM();
    }

    /**
     * Default constructor for Flywheel subsystem using default configuration.
     */
    public Flywheel() {
        this(new PollenShooterConfig());
    }

    /**
     * Sets the target velocity for the flywheel in RPM.
     *
     * @param targetRPM Target revolutions per minute
     */
    public void setTargetRPM(double targetRPM) {
        if (targetRPM > maxRPM) targetRPM = maxRPM;
        if (targetRPM < 0) targetRPM = 0;
        this.targetRPM = targetRPM;
    }

    /**
     * Gets the configured target RPM.
     *
     * @return Target RPM
     */
    public double getTargetRPM() {
        return targetRPM;
    }

    /**
     * Gets the measured current velocity in RPM.
     *
     * @return Measured RPM
     */
    public double getCurrentRPM() {
        return currentRPM;
    }

    /**
     * Checks if the flywheel speed is within acceptable tolerance of the target RPM.
     *
     * @return True if flywheel speed is at target
     */
    public boolean isAtTargetSpeed() {
        return Math.abs(currentRPM - targetRPM) <= toleranceRPM;
    }

    /**
     * Sets the tolerance for target speed matching.
     *
     * @param toleranceRPM RPM tolerance threshold
     */
    public void setToleranceRPM(double toleranceRPM) {
        this.toleranceRPM = toleranceRPM;
    }

    /**
     * Stops flywheel rotation.
     */
    public void stop() {
        setTargetRPM(0.0);
    }

    public String getMotor1Name() { return motor1Name; }
    public String getMotor2Name() { return motor2Name; }
    public double getKp() { return kp; }
    public double getKi() { return ki; }
    public double getKd() { return kd; }
    public double getKf() { return kf; }
    public double getMaxRPM() { return maxRPM; }

    @Override
    public void periodic() {
        // Stub: Execute PID velocity control loop, motor power assignments, or updates
    }
}
