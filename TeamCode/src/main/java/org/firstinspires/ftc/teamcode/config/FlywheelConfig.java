package org.firstinspires.ftc.teamcode.config;

import com.acmerobotics.dashboard.config.Config;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;

@Config
public class FlywheelConfig {
    public static String FLYWHEEL1_NAME = "idk"; //TODO: change flywheel motor names
    public static String FLYWHEEL2_NAME = "idk";
    public static MotorEx.GoBILDA FLYWHEEL_MOTOR = MotorEx.GoBILDA.BARE; //TODO: set correct flywheel motor type
    public static boolean FLYWHEEL1_INVERTED = false;
    public static boolean FLYWHEEL2_INVERTED = true;
    public static double FLYWHEEL_GEAR_RATIO = 1.0;

    // --- PID / Feedforward ---
    public static double FLYWHEEL_KP = 0;
    public static double FLYWHEEL_KP_DOWN = 0;
    public static double FLYWHEEL_KI = 0;
    public static double FLYWHEEL_KD = 0;
    public static double FLYWHEEL_KS = 0;
    public static double FLYWHEEL_KV = 0;
    public static double FLYWHEEL_KA = 0;
    public static double FLYWHEEL_KA_DOWN = 0;
    public static double FLYWHEEL_DELAY_SEC = 0;

    // --- Asymmetric gain (brake vs. drive) ---
    public static double BRAKE_ENTRY_THRESHOLD = -50;
    public static double BRAKE_EXIT_THRESHOLD = -20;
    public static double INITIAL_RAMP_DURATION = 0.5;

    // --- Safety ---
    public static double CURRENT_THRESHOLD = 8;
    public static double STALL_TIMEOUT = 1.0;
    public static double RPM_REACHED_THRESHOLD = 50;

    // --- Filtering ---
    public static int RPM_WINDOW_SIZE = 5;
    public static double FLYWHEEL_ACCEL_FILTER_ALPHA = 0.3;
    public static double FLYWHEEL_SHOOTING_DIFFERENCE = 0;
}
