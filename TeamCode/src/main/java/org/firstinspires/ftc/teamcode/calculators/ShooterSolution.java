package org.firstinspires.ftc.teamcode.calculators;

/**
 * Data container class holding calculated target parameters for the shooter subsystem.
 * Encapsulates flywheel velocity/RPM, turret orientation angle, and hood elevation angle.
 */
public class ShooterSolution {
    private final double flywheelRPM;
    private final double turretAngleDegrees;
    private final double hoodAngleDegrees;

    /**
     * Constructs a target solution container.
     *
     * @param flywheelRPM Target speed/RPM for the flywheel
     * @param turretAngleDegrees Target yaw angle for the turret in degrees
     * @param hoodAngleDegrees Target pitch angle for the hood in degrees
     */
    public ShooterSolution(double flywheelRPM, double turretAngleDegrees, double hoodAngleDegrees) {
        this.flywheelRPM = flywheelRPM;
        this.turretAngleDegrees = turretAngleDegrees;
        this.hoodAngleDegrees = hoodAngleDegrees;
    }

    public double getFlywheelRPM() {
        return flywheelRPM;
    }

    public double getTurretAngleDegrees() {
        return turretAngleDegrees;
    }

    public double getHoodAngleDegrees() {
        return hoodAngleDegrees;
    }


    @Override
    public String toString() {
        return String.format("ShooterSolution{RPM=%.1f, Turret=%.2f deg, Hood=%.2f deg, Dist=%.2f}",
                flywheelRPM, turretAngleDegrees, hoodAngleDegrees);
    }
}
