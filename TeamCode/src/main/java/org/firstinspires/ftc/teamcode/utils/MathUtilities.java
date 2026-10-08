package org.firstinspires.ftc.teamcode.utils;

import com.seattlesolvers.solverslib.util.MathUtils;

public class MathUtilities {
    private MathUtilities() {}

    public static double lowPassFilter(double newVal, double oldVal, double gain) {
        return (gain * newVal) + ((1.0 - gain) * oldVal);
    }
}
