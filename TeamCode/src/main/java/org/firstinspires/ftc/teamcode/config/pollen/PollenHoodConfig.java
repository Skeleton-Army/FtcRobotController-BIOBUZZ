package org.firstinspires.ftc.teamcode.config.pollen;

import com.acmerobotics.dashboard.config.Config;
import org.firstinspires.ftc.teamcode.config.interfaces.HoodConfig;
import org.firstinspires.ftc.teamcode.utils.Polynomial;

@Config
public class PollenHoodConfig implements HoodConfig {
    public static String HOOD_NAME = "hood";
    public static boolean HOOD_INVERTED = true;

    // --- Servo position range the hardware can physically reach ---
    public static double HOOD_POSSIBLE_MIN = 0.2;
    public static double HOOD_POSSIBLE_MAX = 1.0;

    // --- Physical angle range (radians) the servo range maps to ---
    public static double HOOD_MIN = 0.0;
    public static double HOOD_MAX = Math.toRadians(45);

    // --- Usable angle range (radians), may be narrower than HOOD_MIN/MAX ---
    public static double HOOD_USABLE_MIN = 0.0;
    public static double HOOD_USABLE_MAX = Math.toRadians(45);
    public static double HOOD_DEGREE_RATIO = 1.0;

    // --- Shooter ---
    public static Polynomial rpmModel = Polynomial.of(87.25612603, -16.43487126, 3.72896588, -0.80113754);

    public PollenHoodConfig() {}

    @Override
    public String getHoodName() {
        return HOOD_NAME;
    }

    @Override
    public boolean isHoodInverted() {
        return HOOD_INVERTED;
    }

    @Override
    public double getHoodPossibleMin() {
        return HOOD_POSSIBLE_MIN;
    }

    @Override
    public double getHoodPossibleMax() {
        return HOOD_POSSIBLE_MAX;
    }

    @Override
    public double getHoodMin() {
        return HOOD_MIN;
    }

    @Override
    public double getHoodMax() {
        return HOOD_MAX;
    }

    @Override
    public double getHoodUsableMin() {
        return HOOD_USABLE_MIN;
    }

    @Override
    public double getHoodUsableMax() {
        return HOOD_USABLE_MAX;
    }

    @Override
    public double getHoodDegreeRatio() {
        return HOOD_DEGREE_RATIO;
    }

    @Override
    public Polynomial getModel() {
        return rpmModel;
    }
}
