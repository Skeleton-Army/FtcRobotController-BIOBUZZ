package org.firstinspires.ftc.teamcode.subsystems;

import com.seattlesolvers.solverslib.command.SubsystemBase;
import org.firstinspires.ftc.teamcode.config.PollenShooterConfig;
import org.firstinspires.ftc.teamcode.config.ShooterConfig;

/**
 * Subsystem placeholder for controlling the turret yaw orientation.
 * Responsible for rotating the launcher assembly horizontally.
 */
public class Turret extends SubsystemBase {

    private String motorName = "turretMotor";
    private double kp = 0.03;
    private double ki = 0.0;
    private double kd = 0.002;
    private double ticksPerDegree = 15.55;
    private double minAngleDegrees = -180.0;
    private double maxAngleDegrees = 180.0;

    private double targetAngleDegrees = 0.0;
    private double currentAngleDegrees = 0.0;
    private double toleranceDegrees = 1.0;

    /**
     * Constructs a Turret subsystem configured using constants from a {@link ShooterConfig}.
     * Throws an {@link IllegalArgumentException} if config is null.
     *
     * @param config Configuration containing turret parameters
     */
    public Turret(ShooterConfig config) {
        setName("TurretSubsystem");
        if (config == null) {
            throw new IllegalArgumentException("ShooterConfig cannot be null");
        }
        this.motorName = config.getTurretMotorName();
        this.kp = config.getTurretKp();
        this.ki = config.getTurretKi();
        this.kd = config.getTurretKd();
        this.ticksPerDegree = config.getTurretTicksPerDegree();
        this.minAngleDegrees = config.getMinTurretAngle();
        this.maxAngleDegrees = config.getMaxTurretAngle();
    }

    /**
     * Default constructor for Turret subsystem using default configuration.
     */
    public Turret() {
        this(new PollenShooterConfig());
    }

    /**
     * Sets the target heading/yaw angle for the turret in degrees.
     *
     * @param angleDegrees Target angle in degrees
     */
    public void setTargetAngle(double angleDegrees) {
        if (angleDegrees < minAngleDegrees) angleDegrees = minAngleDegrees;
        if (angleDegrees > maxAngleDegrees) angleDegrees = maxAngleDegrees;
        this.targetAngleDegrees = angleDegrees;
    }

    /**
     * Gets the current target angle setting for the turret.
     *
     * @return Target angle in degrees
     */
    public double getTargetAngle() {
        return targetAngleDegrees;
    }

    /**
     * Gets the estimated/measured current turret position in degrees.
     *
     * @return Current angle in degrees
     */
    public double getCurrentAngle() {
        return currentAngleDegrees;
    }

    /**
     * Checks if the turret is at the target angle within tolerance.
     *
     * @return True if target angle reached
     */
    public boolean isAtTargetAngle() {
        return Math.abs(currentAngleDegrees - targetAngleDegrees) <= toleranceDegrees;
    }

    /**
     * Sets position tolerance in degrees.
     *
     * @param toleranceDegrees Acceptable angle error margin
     */
    public void setTolerance(double toleranceDegrees) {
        this.toleranceDegrees = toleranceDegrees;
    }

    /**
     * Resets turret orientation to home/zero position.
     */
    public void homePosition() {
        this.targetAngleDegrees = 0.0;
        this.currentAngleDegrees = 0.0;
    }

    public String getMotorName() { return motorName; }
    public double getKp() { return kp; }
    public double getKi() { return ki; }
    public double getKd() { return kd; }
    public double getTicksPerDegree() { return ticksPerDegree; }

    @Override
    public void periodic() {
        // Stub: Update control loop output, feedback sensors, or telemetry here
    }
}
