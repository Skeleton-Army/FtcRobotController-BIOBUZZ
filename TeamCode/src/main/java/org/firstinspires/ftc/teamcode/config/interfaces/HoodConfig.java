package org.firstinspires.ftc.teamcode.config.interfaces;

import org.firstinspires.ftc.teamcode.utils.Polynomial;

public interface HoodConfig {
    String getHoodName();

    boolean isHoodInverted();

    double getHoodPossibleMin();

    double getHoodPossibleMax();

    double getHoodMin();

    double getHoodMax();

    double getHoodUsableMin();

    double getHoodUsableMax();

    double getHoodDegreeRatio();

    Polynomial getModel();
}
