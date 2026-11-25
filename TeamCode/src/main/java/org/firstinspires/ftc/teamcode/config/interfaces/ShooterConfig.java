package org.firstinspires.ftc.teamcode.config.interfaces;

/**
 * Base configuration interface containing subsystem parameters such as polynomial models,
 * device hardware identifiers/names, control loop constants (PID/feedforward),
 * and mechanical conversion factors.
 */
public interface ShooterConfig {
    FlywheelConfig getFlywheelConfig();

    HoodConfig getHoodConfig();

    TurretConfig getTurretConfig();
}
