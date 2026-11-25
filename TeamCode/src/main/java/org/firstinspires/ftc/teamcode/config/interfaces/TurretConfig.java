package org.firstinspires.ftc.teamcode.config.interfaces;

import com.seattlesolvers.solverslib.hardware.motors.Motor;

public interface TurretConfig {
    String getTurretName();

    Motor.GoBILDA getTurretMotor();

    double getGearRatio();

    double getTurretKp();

    double getTurretKi();

    double getTurretKd();

    double getTurretKv();

    double getTurretKa();

    double getTurretKs();

    double getTurretDerivativeGain();

    double getTurretSecondDerivativeGain();

    double getTurretIzone();

    double getTurretMinVoltage();

    double getMinUpdateDt();

    double getStartupSettleTime();

    double getMaxTargetVel();

    double getMaxTargetAccel();

    double getTurretMin();

    double getTurretMax();

    boolean isTurretWrap();

    double getTurretPositionTolerance();

    double getTurretVelocityTolerance();

    double getTurretOffsetX();

    double getTurretOffsetY();

    double getStallMinVelocity();

    double getStallMinPower();

    double getTravelTimeThreshold();
}
