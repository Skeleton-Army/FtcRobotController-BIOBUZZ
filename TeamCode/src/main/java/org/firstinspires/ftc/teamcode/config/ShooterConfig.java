package org.firstinspires.ftc.teamcode.config;

import org.firstinspires.ftc.teamcode.utilities.Polynomial;

/**
 * Base configuration class containing subsystem parameters such as polynomial models,
 * device hardware identifiers/names, control loop constants (PID/feedforward),
 * and mechanical conversion factors.
 */
public class ShooterConfig {

    // Subsystem hardware identifiers / names
    protected String turretMotorName;
    protected String flywheelMotor1Name;
    protected String flywheelMotor2Name;
    protected String hoodServoName;

    // Control constants (PID and Feedforward gains)
    protected double flywheelKp = 0.005;
    protected double flywheelKi = 0.0;
    protected double flywheelKd = 0.0001;
    protected double flywheelKf = 0.0002;

    protected double turretKp = 0.03;
    protected double turretKi = 0.0;
    protected double turretKd = 0.002;

    protected double hoodKp = 0.02;
    protected double hoodKi = 0.0;
    protected double hoodKd = 0.001;

    // Mechanical conversion factors
    protected double flywheelTicksPerRev = 28.0;
    protected double turretTicksPerDegree = 15.55;
    protected double hoodServoDegreeRatio = 1.0;
    protected double maxFlywheelRPM = 6000.0;
    protected double maxTurretAngle = 180.0;
    protected double minTurretAngle = -180.0;
    protected double maxHoodAngle = 60.0;
    protected double minHoodAngle = 0.0;

    // Polynomial calculation models
    protected Polynomial flywheelRpmModel = Polynomial.linear(2000.0, 15.0);
    protected Polynomial hoodAngleModel = Polynomial.constant(25.0);

    public ShooterConfig() {
    }

    // Getters and Setters for hardware names
    public String getTurretMotorName() {
        return turretMotorName;
    }

    public void setTurretMotorName(String turretMotorName) {
        this.turretMotorName = turretMotorName;
    }

    public String getFlywheelMotor1Name() {
        return flywheelMotor1Name;
    }

    public void setFlywheelMotor1Name(String flywheelMotor1Name) {
        this.flywheelMotor1Name = flywheelMotor1Name;
    }

    public String getFlywheelMotor2Name() {
        return flywheelMotor2Name;
    }

    public void setFlywheelMotor2Name(String flywheelMotor2Name) {
        this.flywheelMotor2Name = flywheelMotor2Name;
    }

    public String getHoodServoName() {
        return hoodServoName;
    }

    public void setHoodServoName(String hoodServoName) {
        this.hoodServoName = hoodServoName;
    }

    // Getters and Setters for PID constants
    public double getFlywheelKp() { return flywheelKp; }
    public double getFlywheelKi() { return flywheelKi; }
    public double getFlywheelKd() { return flywheelKd; }
    public double getFlywheelKf() { return flywheelKf; }

    public double getTurretKp() { return turretKp; }
    public double getTurretKi() { return turretKi; }
    public double getTurretKd() { return turretKd; }

    public double getHoodKp() { return hoodKp; }
    public double getHoodKi() { return hoodKi; }
    public double getHoodKd() { return hoodKd; }

    // Getters and Setters for conversion factors
    public double getFlywheelTicksPerRev() { return flywheelTicksPerRev; }
    public double getTurretTicksPerDegree() { return turretTicksPerDegree; }
    public double getHoodServoDegreeRatio() { return hoodServoDegreeRatio; }
    public double getMaxFlywheelRPM() { return maxFlywheelRPM; }
    public double getMaxTurretAngle() { return maxTurretAngle; }
    public double getMinTurretAngle() { return minTurretAngle; }
    public double getMaxHoodAngle() { return maxHoodAngle; }
    public double getMinHoodAngle() { return minHoodAngle; }

    // Getters and Setters for Polynomial models
    public Polynomial getFlywheelRpmModel() {
        return flywheelRpmModel;
    }

    public void setFlywheelRpmModel(Polynomial flywheelRpmModel) {
        this.flywheelRpmModel = flywheelRpmModel;
    }

    public Polynomial getHoodAngleModel() {
        return hoodAngleModel;
    }

    public void setHoodAngleModel(Polynomial hoodAngleModel) {
        this.hoodAngleModel = hoodAngleModel;
    }
}
