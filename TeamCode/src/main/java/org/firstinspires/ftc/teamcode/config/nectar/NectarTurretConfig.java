package org.firstinspires.ftc.teamcode.config.nectar;

import com.seattlesolvers.solverslib.hardware.motors.Motor;
import org.firstinspires.ftc.teamcode.config.interfaces.TurretConfig;

public class NectarTurretConfig implements TurretConfig {
    public NectarTurretConfig() {}

    @Override
    public String getTurretName() {
        return "";
    }

    @Override
    public Motor.GoBILDA getTurretMotor() {
        return null;
    }

    @Override
    public double getGearRatio() {
        return 0;
    }

    @Override
    public double getTurretKp() {
        return 0;
    }

    @Override
    public double getTurretKi() {
        return 0;
    }

    @Override
    public double getTurretKd() {
        return 0;
    }

    @Override
    public double getTurretKv() {
        return 0;
    }

    @Override
    public double getTurretKa() {
        return 0;
    }

    @Override
    public double getTurretKs() {
        return 0;
    }

    @Override
    public double getTurretDerivativeGain() {
        return 0;
    }

    @Override
    public double getTurretSecondDerivativeGain() {
        return 0;
    }

    @Override
    public double getTurretIzone() {
        return 0;
    }

    @Override
    public double getTurretMinVoltage() {
        return 0;
    }

    @Override
    public double getMinUpdateDt() {
        return 0;
    }

    @Override
    public double getStartupSettleTime() {
        return 0;
    }

    @Override
    public double getMaxTargetVel() {
        return 0;
    }

    @Override
    public double getMaxTargetAccel() {
        return 0;
    }

    @Override
    public double getTurretMin() {
        return 0;
    }

    @Override
    public double getTurretMax() {
        return 0;
    }

    @Override
    public boolean isTurretWrap() {
        return false;
    }

    @Override
    public double getTurretPositionTolerance() {
        return 0;
    }

    @Override
    public double getTurretVelocityTolerance() {
        return 0;
    }

    @Override
    public double getTurretOffsetX() {
        return 0;
    }

    @Override
    public double getTurretOffsetY() {
        return 0;
    }

    @Override
    public double getStallMinVelocity() {
        return 0;
    }

    @Override
    public double getStallMinPower() {
        return 0;
    }

    @Override
    public double getTravelTimeThreshold() {
        return 0;
    }
}
