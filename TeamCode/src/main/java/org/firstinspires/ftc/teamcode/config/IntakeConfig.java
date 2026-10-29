package org.firstinspires.ftc.teamcode.config;

import com.acmerobotics.dashboard.config.Config;
import com.seattlesolvers.solverslib.hardware.motors.Motor;

@Config
public class IntakeConfig {
    public static String INTAKE_NAME = "intake";
    public static Motor.GoBILDA INTAKE_MOTOR = Motor.GoBILDA.RPM_1150;
    public static double INTAKE_RELEASE_POWER = 1;
}