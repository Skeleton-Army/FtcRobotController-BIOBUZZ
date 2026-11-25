package org.firstinspires.ftc.teamcode.config.nectar;

import com.seattlesolvers.solverslib.hardware.motors.Motor;
import org.firstinspires.ftc.teamcode.config.interfaces.FlywheelConfig;
import org.firstinspires.ftc.teamcode.utils.Polynomial;

public class NectarFlywheelConfig implements FlywheelConfig {
    public NectarFlywheelConfig() {}

    @Override
    public String getFlywheelName() {
        return "";
    }

    @Override
    public Motor.GoBILDA getFlywheelMotor() {
        return null;
    }

    @Override
    public boolean isFlywheelInverted() {
        return false;
    }

    @Override
    public double getFlywheelGearRatio() {
        return 0;
    }

    @Override
    public double getFlywheelKp() {
        return 0;
    }

    @Override
    public double getFlywheelKi() {
        return 0;
    }

    @Override
    public double getFlywheelKd() {
        return 0;
    }

    @Override
    public double getFlywheelKs() {
        return 0;
    }

    @Override
    public double getFlywheelKv() {
        return 0;
    }

    @Override
    public double getFlywheelKa() {
        return 0;
    }

    @Override
    public double getStallTimeout() {
        return 0;
    }

    @Override
    public double getRpmReachedThreshold() {
        return 0;
    }

    @Override
    public double getWrongDirectionRpmThreshold() {
        return 0;
    }

    @Override
    public double getStallTargetRpmThreshold() {
        return 0;
    }

    @Override
    public double getStallActualRpmThreshold() {
        return 0;
    }

    @Override
    public double getInitialRampDuration() {
        return 0;
    }

    @Override
    public double getFlywheelAccelFilterAlpha() {
        return 0;
    }

    @Override
    public double getMinDtSeconds() {
        return 0;
    }

    @Override
    public Polynomial getModel() {
        return null;
    }
}
