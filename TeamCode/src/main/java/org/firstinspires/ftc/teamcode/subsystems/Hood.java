package org.firstinspires.ftc.teamcode.subsystems;

import androidx.core.math.MathUtils;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.hardware.servos.ServoEx;
import org.firstinspires.ftc.teamcode.config.interfaces.HoodConfig;

/**
 * Subsystem placeholder for controlling the shooter hood angle (vertical launch pitch).
 */
public class Hood extends SubsystemBase {
    private final ServoEx hood;
    private final HoodConfig config;

    public Hood(final HardwareMap hardwareMap, HoodConfig hoodConfig) {
        super();
        config = hoodConfig;
        // Measurements given by hardware, if problem blame Tzipi.
        hood = new ServoEx(hardwareMap, config.getHoodName());
    }

    public double getRawHoodPosition() {
        return hood.getRawPosition();
    }

    /**
     * Calculates the current physical angle of the hood based on the servo's position.
     * @return physical angle in radians
     */
    public double getHoodAngle() {
        double pos = getRawHoodPosition();

        double normalizedPos =
                (pos - config.getHoodPossibleMin()) / (config.getHoodPossibleMax() - config.getHoodPossibleMin());

        if (config.isHoodInverted()) {
            normalizedPos = 1.0 - normalizedPos;
        }

        return config.getHoodMin() + (normalizedPos * (config.getHoodMax() - config.getHoodMin()));
    }

    public double getHoodAngleDegrees() {
        return Math.toDegrees(getHoodAngle());
    }

    /**
     * Sets the hood angle by interpolating the target radians between the
     * physical min/max angles and the servo's possible position range.
     *
     * @param angleRad the target hood angle in radians
     */
    public void setVerticalAngle(double angleRad) {
        double clampedAngle = MathUtils.clamp(angleRad, config.getHoodUsableMin(), config.getHoodUsableMax());

        double t = (clampedAngle - config.getHoodMin()) / (config.getHoodMax() - config.getHoodMin());

        double targetPos =
                config.getHoodPossibleMin() + (t * (config.getHoodPossibleMax() - config.getHoodPossibleMin()));

        if (config.isHoodInverted()) {
            targetPos = config.getHoodPossibleMax() - (targetPos - config.getHoodPossibleMin());
        }

        hood.set(targetPos);
    }

    public boolean isAtTarget() {
        return false; // TODO: actually calculate
    }

    public void home() {
        hood.set(config.getHoodMin());
    }
}
