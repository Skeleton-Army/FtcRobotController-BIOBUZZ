package org.firstinspires.ftc.teamcode.config;

import com.acmerobotics.dashboard.config.Config;
import com.seattlesolvers.solverslib.hardware.motors.Motor;

@Config
public class TransferConfig {
    public static String TRANSFER_NAME = "transfer";
    public static Motor.GoBILDA TRANSFER_MOTOR = Motor.GoBILDA.RPM_1150;
    public static double TRANSFER_POWER = 0.7;
    public static String STOPPERS_NAME = "stopper";
    public static double STOPPER_MIN = .5;
    public static double STOPPER_MAX = 1;
}
