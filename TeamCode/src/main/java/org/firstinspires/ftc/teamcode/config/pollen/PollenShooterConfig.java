package org.firstinspires.ftc.teamcode.config.pollen;

import org.firstinspires.ftc.teamcode.config.interfaces.FlywheelConfig;
import org.firstinspires.ftc.teamcode.config.interfaces.HoodConfig;
import org.firstinspires.ftc.teamcode.config.interfaces.ShooterConfig;
import org.firstinspires.ftc.teamcode.config.interfaces.TurretConfig;

/**
 * Concrete configuration subclass for standard TeleOp operation.
 */
public class PollenShooterConfig implements ShooterConfig {
    private final HoodConfig hoodConfig;
    private final TurretConfig turretConfig;
    private final FlywheelConfig flywheelConfig;

    /*
    public PollenShooterConfig() {
        super();
        this.turretMotorName = "turretMotor";
        this.flywheelMotor1Name = "flywheelLeft";
        this.flywheelMotor2Name = "flywheelRight";
        this.hoodServoName = "hoodServo";

        this.flywheelKp = 0.005;
        this.flywheelKi = 0.0001;
        this.flywheelKd = 0.0002;
        this.flywheelKf = 0.00018;

        // this.flywheelRpmModel = Polynomial.of(4.8403378378, 0.7418918919);
        this.flywheelRpmModel = Polynomial.of(0.7418918919, 4.8403378378);
        this.hoodAngleModel = Polynomial.of(87.25612603, -16.43487126, 3.72896588, -0.80113754);
        // this.hoodAngleModel = Polynomial.of(-0.80113754, 3.72896588, -16.43487126, 87.25612603);
    }

     */

    public PollenShooterConfig() {
        this.hoodConfig = new PollenHoodConfig();
        this.flywheelConfig = new PollenFlywheelConfig();
        this.turretConfig = new PollenTurretConfig();
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
