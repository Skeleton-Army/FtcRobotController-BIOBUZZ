package org.firstinspires.ftc.teamcode.config.interfaces;

import com.seattlesolvers.solverslib.hardware.motors.Motor;
import org.firstinspires.ftc.teamcode.utils.Polynomial;

public interface FlywheelConfig {
    String getFlywheelName();

    Motor.GoBILDA getFlywheelMotor();

    boolean isFlywheelInverted();

    double getFlywheelGearRatio();

    double getFlywheelKp();

    double getFlywheelKi();

    double getFlywheelKd();

    double getFlywheelKs();

    double getFlywheelKv();

    double getFlywheelKa();

    double getStallTimeout();

    double getRpmReachedThreshold();

    double getWrongDirectionRpmThreshold();

    double getStallTargetRpmThreshold();

    double getStallActualRpmThreshold();

    double getInitialRampDuration();

    double getFlywheelAccelFilterAlpha();

    double getMinDtSeconds();

    Polynomial getModel();
}
