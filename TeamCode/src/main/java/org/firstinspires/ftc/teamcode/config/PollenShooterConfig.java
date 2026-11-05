package org.firstinspires.ftc.teamcode.config;

import org.firstinspires.ftc.teamcode.utilities.Polynomial;

/**
 * Concrete configuration subclass for standard TeleOp operation.
 */
public class PollenShooterConfig extends ShooterConfig {

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

        this.flywheelRpmModel = Polynomial.of(1800.0, 12.5, 0.02);
        this.hoodAngleModel = Polynomial.of(20.0, 0.15);
    }
}
