package org.firstinspires.ftc.teamcode.config;

import org.firstinspires.ftc.teamcode.utilities.Polynomial;

/**
 * Concrete configuration subclass optimized for high-velocity long-range target profiles.
 */
public class NectarShooterConfig extends ShooterConfig {

    public NectarShooterConfig() {
        super();
        this.flywheelKp = 0.008;
        this.flywheelKi = 0.0002;
        this.flywheelKd = 0.0003;
        this.flywheelKf = 0.00022;

        this.maxFlywheelRPM = 6500.0;

        // Quadratic flywheel model for long-range high-velocity trajectories
        this.flywheelRpmModel = Polynomial.of(2500.0, 18.0, 0.05);
        this.hoodAngleModel = Polynomial.of(30.0, 0.25);
    }
}
