package org.firstinspires.ftc.teamcode.config;

import com.acmerobotics.dashboard.config.Config;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;

@Config
public class FlywheelConfig {
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
}
