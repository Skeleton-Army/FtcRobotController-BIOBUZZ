package org.firstinspires.ftc.teamcode.config.pollen;

import com.acmerobotics.dashboard.config.Config;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import org.firstinspires.ftc.teamcode.config.interfaces.TurretConfig;

@Config
public class PollenTurretConfig implements TurretConfig {
    public static String TURRET_NAME = "turret";
    public static Motor.GoBILDA TURRET_MOTOR = Motor.GoBILDA.RPM_1150;
    public static double GEAR_RATIO = (double) 111 / 11;

    // --- PID ---
    public static double TURRET_KP = 12;
    public static double TURRET_KI = 2;
    public static double TURRET_KD = 0.3;
    public static double TURRET_KV = 1;
    public static double TURRET_KA = 0;
    public static double TURRET_KS = 0.5;
    public static double TURRET_DERIVATIVE_GAIN = 0.8;
    public static double TURRET_SECOND_DERIVATIVE_GAIN = 0.8;
    public static double TURRET_IZONE = Math.toRadians(30);
    public static double TURRET_MIN_VOLTAGE = 0.2;
    public static final double MIN_UPDATE_DT = 0.005;
    public static final double STARTUP_SETTLE_TIME = 0.5;
    public static final double MAX_TARGET_VEL = 4;
    public static final double MAX_TARGET_ACCEL = 4;

    // --- Range / wrapping ---
    public static double TURRET_MIN = Math.toRadians(-90);
    public static double TURRET_MAX = Math.toRadians(90);
    public static boolean TURRET_WRAP = true;

    // --- Tolerance ---
    public static double TURRET_POSITION_TOLERANCE = 0.02;
    public static double TURRET_VELOCITY_TOLERANCE = 0.1;

    // -- Geometry --
    public static double TURRET_OFFSET_X = 0;
    public static double TURRET_OFFSET_Y = 0;

    // -- Safety --
    public static double STALL_MIN_VELOCITY = 0.05;
    public static double STALL_MIN_POWER = 0.3;
    public static double TRAVEL_TIME_THRESHOLD = 0.5;

    public PollenTurretConfig() {}

    @Override
    public String getTurretName() {
        return TURRET_NAME;
    }

    @Override
    public Motor.GoBILDA getTurretMotor() {
        return TURRET_MOTOR;
    }

    @Override
    public double getGearRatio() {
        return GEAR_RATIO;
    }

    @Override
    public double getTurretKp() {
        return TURRET_KP;
    }

    @Override
    public double getTurretKi() {
        return TURRET_KI;
    }

    @Override
    public double getTurretKd() {
        return TURRET_KD;
    }

    @Override
    public double getTurretKv() {
        return TURRET_KV;
    }

    @Override
    public double getTurretKa() {
        return TURRET_KA;
    }

    @Override
    public double getTurretKs() {
        return TURRET_KS;
    }

    @Override
    public double getTurretDerivativeGain() {
        return TURRET_DERIVATIVE_GAIN;
    }

    @Override
    public double getTurretSecondDerivativeGain() {
        return TURRET_SECOND_DERIVATIVE_GAIN;
    }

    @Override
    public double getTurretIzone() {
        return TURRET_IZONE;
    }

    @Override
    public double getTurretMinVoltage() {
        return TURRET_MIN_VOLTAGE;
    }

    @Override
    public double getMinUpdateDt() {
        return MIN_UPDATE_DT;
    }

    @Override
    public double getStartupSettleTime() {
        return STARTUP_SETTLE_TIME;
    }

    @Override
    public double getMaxTargetVel() {
        return MAX_TARGET_VEL;
    }

    @Override
    public double getMaxTargetAccel() {
        return MAX_TARGET_ACCEL;
    }

    @Override
    public double getTurretMin() {
        return TURRET_MIN;
    }

    @Override
    public double getTurretMax() {
        return TURRET_MAX;
    }

    @Override
    public boolean isTurretWrap() {
        return TURRET_WRAP;
    }

    @Override
    public double getTurretPositionTolerance() {
        return TURRET_POSITION_TOLERANCE;
    }

    @Override
    public double getTurretVelocityTolerance() {
        return TURRET_VELOCITY_TOLERANCE;
    }

    @Override
    public double getTurretOffsetX() {
        return TURRET_OFFSET_X;
    }

    @Override
    public double getTurretOffsetY() {
        return TURRET_OFFSET_Y;
    }

    @Override
    public double getStallMinVelocity() {
        return STALL_MIN_VELOCITY;
    }

    @Override
    public double getStallMinPower() {
        return STALL_MIN_POWER;
    }

    @Override
    public double getTravelTimeThreshold() {
        return TRAVEL_TIME_THRESHOLD;
    }
}
