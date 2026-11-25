package org.firstinspires.ftc.teamcode.config.nectar;

import org.firstinspires.ftc.teamcode.config.interfaces.FlywheelConfig;
import org.firstinspires.ftc.teamcode.config.interfaces.HoodConfig;
import org.firstinspires.ftc.teamcode.config.interfaces.ShooterConfig;
import org.firstinspires.ftc.teamcode.config.interfaces.TurretConfig;

/**
 * Concrete configuration subclass optimized for high-velocity long-range target profiles.
 */
public class NectarShooterConfig implements ShooterConfig {
    private final FlywheelConfig flywheelConfig;
    private final HoodConfig hoodConfig;
    private final TurretConfig turretConfig;

    public NectarShooterConfig() {

        this.flywheelConfig = new NectarFlywheelConfig();
        this.hoodConfig = new NectarHoodConfig();
        this.turretConfig = new NectarTurretConfig();
    }

    @Override
    public FlywheelConfig getFlywheelConfig() {
        return flywheelConfig;
    }

    @Override
    public HoodConfig getHoodConfig() {
        return hoodConfig;
    }

    @Override
    public TurretConfig getTurretConfig() {
        return turretConfig;
    }
}
