package org.firstinspires.ftc.teamcode.subsystems;

import com.seattlesolvers.solverslib.command.SubsystemBase;
import org.firstinspires.ftc.teamcode.config.PollenShooterConfig;
import org.firstinspires.ftc.teamcode.config.ShooterConfig;

/**
 * Subsystem placeholder for controlling the shooter hood angle (vertical launch pitch).
 */
public class Hood extends SubsystemBase {

    private String servoName;
    private double kp;
    private double ki;
    private double kd;
    private double degreeRatio;
    private double minAngleDegrees;
    private double maxAngleDegrees;

    private double targetAngleDegrees = 0.0;
    private double currentAngleDegrees = 0.0;
    private double toleranceDegrees = 0.5;

    /**
     * Constructs a Hood subsystem configured using constants from a {@link ShooterConfig}.
     * Throws an {@link IllegalArgumentException} if config is null.
     *
     * @param config Configuration containing hood parameters
     */
    public Hood(ShooterConfig config) {
        setName("HoodSubsystem");
        if (config == null) {
            throw new IllegalArgumentException("ShooterConfig cannot be null");
        }
        this.servoName = config.getHoodServoName();
        this.kp = config.getHoodKp();
        this.ki = config.getHoodKi();
        this.kd = config.getHoodKd();
        this.degreeRatio = config.getHoodServoDegreeRatio();
        this.minAngleDegrees = config.getMinHoodAngle();
        this.maxAngleDegrees = config.getMaxHoodAngle();
    }

    /**
     * Default constructor for Hood subsystem using default configuration.
     */
    public Hood() {
        this(new PollenShooterConfig());
    }

    /**
     * Sets the target hood angle in degrees.
     *
     * @param angleDegrees Target pitch angle in degrees
     */
    public void setTargetAngle(double angleDegrees) {
        if (angleDegrees < minAngleDegrees) angleDegrees = minAngleDegrees;
        if (angleDegrees > maxAngleDegrees) angleDegrees = maxAngleDegrees;
        this.targetAngleDegrees = angleDegrees;
    }

    /**
     * Gets the configured target angle in degrees.
     *
     * @return Target pitch angle in degrees
     */
    public double getTargetAngle() {
        return targetAngleDegrees;
    }

    /**
     * Gets the current measured pitch angle in degrees.
     *
     * @return Current pitch angle in degrees
     */
    public double getCurrentAngle() {
        return currentAngleDegrees;
    }

    /**
     * Checks if the hood angle is within tolerance of the target.
     *
     * @return True if target angle reached
     */
    public boolean isAtTargetAngle() {
        return Math.abs(currentAngleDegrees - targetAngleDegrees) <= toleranceDegrees;
    }

    /**
     * Sets angle margin tolerance.
     *
     * @param toleranceDegrees Acceptable angle error margin
     */
    public void setTolerance(double toleranceDegrees) {
        this.toleranceDegrees = toleranceDegrees;
    }

    /**
     * Retracts hood to default minimum position.
     */
    public void retract() {
        setTargetAngle(0.0);
    }

    public String getServoName() { return servoName; }
    public double getKp() { return kp; }
    public double getKi() { return ki; }
    public double getKd() { return kd; }
    public double getDegreeRatio() { return degreeRatio; }

    @Override
    public void periodic() {
        // Stub: Update position servo/motor controller output
    }
}
