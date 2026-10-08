package org.firstinspires.ftc.teamcode.config;

import com.acmerobotics.dashboard.config.Config;
import com.seattlesolvers.solverslib.hardware.motors.Motor;

@Config
public class TurretConfig {
    public static String TURRET_NAME = "idk"; //TODO: change turret motor name
    public static Motor.GoBILDA TURRET_MOTOR = Motor.GoBILDA.RPM_1150; //TODO: set correct turret motor type
    public static boolean TURRET_INVERTED = false;
    public static double GEAR_RATIO = 1.0; //TODO: set turret gear ratio

    // --- PID ---
    public static double TURRET_KP = 0;
    public static double TURRET_KI = 0;
    public static double TURRET_KD = 0;
    public static double TURRET_KV = 0;
    public static double TURRET_KA = 0;
    public static double TURRET_KS = 0;
    public static double TURRET_DERIVATIVE_GAIN = 0;
    public static double TURRET_SECOND_DERIVATIVE_GAIN = 0;
    public static final double MIN_FLYWHEEL_RPM = 0;
    public static double TURRET_IZONE = Math.toRadians(30);
    public static double TURRET_MIN_VOLTAGE = 0.2;
    public static final double MIN_UPDATE_DT = 0.005;
    public static final double STARTUP_SETTLE_TIME = 0.5;
    public static final double MAX_TARGET_VEL = 4;
    public static final double MAX_TARGET_ACCEL = 4;

    // --- Range / wrapping ---
    public static double TURRET_MIN = -Math.PI;
    public static double TURRET_MAX = Math.PI;
    public static boolean TURRET_WRAP = true;

    // --- Tolerance ---
    public static double TURRET_POSITION_TOLERANCE = 0.02;
    public static double TURRET_VELOCITY_TOLERANCE = 0.1;
}