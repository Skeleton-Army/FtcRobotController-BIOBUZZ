package org.firstinspires.ftc.teamcode.config.pollen;

import com.acmerobotics.dashboard.config.Config;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;
import org.firstinspires.ftc.teamcode.config.interfaces.FlywheelConfig;
import org.firstinspires.ftc.teamcode.utils.Polynomial;

@Config
public class PollenFlywheelConfig implements FlywheelConfig {
    public static String FLYWHEEL_NAME = "flywheel";
    public static MotorEx.GoBILDA FLYWHEEL_MOTOR = MotorEx.GoBILDA.BARE;
    public static boolean FLYWHEEL_INVERTED = true;
    public static double FLYWHEEL_GEAR_RATIO = 1.0;

    // --- PID / Feedforward ---
    public static double FLYWHEEL_KP = 0.06;
    public static double FLYWHEEL_KI = 0;
    public static double FLYWHEEL_KD = 0;
    public static double FLYWHEEL_KS = 1.2;
    public static double FLYWHEEL_KV = 0.0049;
    public static double FLYWHEEL_KA = 0.008;

    // --- Safety ---
    public static double STALL_TIMEOUT = 3;
    public static double RPM_REACHED_THRESHOLD = 50;
    public static double WRONG_DIRECTION_RPM_THRESHOLD = 500;
    public static double STALL_TARGET_RPM_THRESHOLD = 1000;
    public static double STALL_ACTUAL_RPM_THRESHOLD = 200;
    public static double INITIAL_RAMP_DURATION = 0.5;

    // --- Filtering ---
    public static double FLYWHEEL_ACCEL_FILTER_ALPHA = 0.3;
    public static double MIN_DT_SECONDS = 0.001;

    // --- Shooting ---
    public static Polynomial rpmModel = Polynomial.of(0.7418918919, 4.8403378378);

    public PollenFlywheelConfig() {}

    @Override
    public String getFlywheelName() {
        return FLYWHEEL_NAME;
    }

    @Override
    public Motor.GoBILDA getFlywheelMotor() {
        return FLYWHEEL_MOTOR;
    }

    @Override
    public boolean isFlywheelInverted() {
        return FLYWHEEL_INVERTED;
    }

    @Override
    public double getFlywheelGearRatio() {
        return FLYWHEEL_GEAR_RATIO;
    }

    @Override
    public double getFlywheelKp() {
        return FLYWHEEL_KP;
    }

    @Override
    public double getFlywheelKi() {
        return FLYWHEEL_KI;
    }

    @Override
    public double getFlywheelKd() {
        return FLYWHEEL_KD;
    }

    @Override
    public double getFlywheelKs() {
        return FLYWHEEL_KS;
    }

    @Override
    public double getFlywheelKv() {
        return FLYWHEEL_KV;
    }

    @Override
    public double getFlywheelKa() {
        return FLYWHEEL_KA;
    }

    @Override
    public double getStallTimeout() {
        return STALL_TIMEOUT;
    }

    @Override
    public double getRpmReachedThreshold() {
        return RPM_REACHED_THRESHOLD;
    }

    @Override
    public double getWrongDirectionRpmThreshold() {
        return WRONG_DIRECTION_RPM_THRESHOLD;
    }

    @Override
    public double getStallTargetRpmThreshold() {
        return STALL_TARGET_RPM_THRESHOLD;
    }

    @Override
    public double getStallActualRpmThreshold() {
        return STALL_ACTUAL_RPM_THRESHOLD;
    }

    @Override
    public double getInitialRampDuration() {
        return INITIAL_RAMP_DURATION;
    }

    @Override
    public double getFlywheelAccelFilterAlpha() {
        return FLYWHEEL_ACCEL_FILTER_ALPHA;
    }

    @Override
    public double getMinDtSeconds() {
        return MIN_DT_SECONDS;
    }

    @Override
    public Polynomial getModel() {
        return rpmModel;
    }
}
