package org.firstinspires.ftc.teamcode.config;

import com.acmerobotics.dashboard.config.Config;

@Config
public class HoodConfig {
    public static String HOOD_NAME = "idk"; //TODO: change hood servo name
    public static boolean HOOD_INVERTED = false;

    // --- Servo position range the hardware can physically reach ---
    public static double HOOD_POSSIBLE_MIN = 0.0;
    public static double HOOD_POSSIBLE_MAX = 1.0;

    // --- Physical angle range (radians) the servo range maps to ---
    public static double HOOD_MIN = 0.0;
    public static double HOOD_MAX = Math.PI / 4;

    // --- Usable angle range (radians), may be narrower than HOOD_MIN/MAX ---
    public static double HOOD_USABLE_MIN = 0.0;
    public static double HOOD_USABLE_MAX = Math.PI / 4;
}
