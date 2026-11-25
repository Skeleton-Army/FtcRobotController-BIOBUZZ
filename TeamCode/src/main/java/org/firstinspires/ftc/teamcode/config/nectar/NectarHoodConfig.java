package org.firstinspires.ftc.teamcode.config.nectar;

import org.firstinspires.ftc.teamcode.config.interfaces.HoodConfig;
import org.firstinspires.ftc.teamcode.utils.Polynomial;

public class NectarHoodConfig implements HoodConfig {
    public NectarHoodConfig() {}

    @Override
    public String getHoodName() {
        return "";
    }

    @Override
    public boolean isHoodInverted() {
        return false;
    }

    @Override
    public double getHoodPossibleMin() {
        return 0;
    }

    @Override
    public double getHoodPossibleMax() {
        return 0;
    }

    @Override
    public double getHoodMin() {
        return 0;
    }

    @Override
    public double getHoodMax() {
        return 0;
    }

    @Override
    public double getHoodUsableMin() {
        return 0;
    }

    @Override
    public double getHoodUsableMax() {
        return 0;
    }

    @Override
    public double getHoodDegreeRatio() {
        return 0;
    }

    @Override
    public Polynomial getModel() {
        return null;
    }
}
