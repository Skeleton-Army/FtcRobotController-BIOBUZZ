package org.firstinspires.ftc.teamcode.calculators;

/**
 * Data container class holding calculated target parameters for the shooter subsystem.
 * Encapsulates flywheel velocity/RPM, turret orientation angle, hood elevation angle,
 * required exit speed, and shot validity.
 */
public class ShooterSolution {
    private final double flywheelRPM;
    private final double turretAngleDegrees;
    private final double hoodAngleDegrees;

    private final boolean isShotPossible;

    /**
     * Constructs a target solution container.
     *
     * @param flywheelRPM Target speed/RPM for the flywheel
     * @param turretAngleDegrees Target yaw angle for the turret in degrees
     * @param hoodAngleDegrees Target pitch angle for the hood in degrees
     * @param isShotPossible True if the hood can reach the angle and speed is valid
     */
    public ShooterSolution(double flywheelRPM, double turretAngleDegrees, double hoodAngleDegrees,
                            boolean isShotPossible) {
        this.flywheelRPM = flywheelRPM;
        this.turretAngleDegrees = turretAngleDegrees;
        this.hoodAngleDegrees = hoodAngleDegrees;
        this.isShotPossible = isShotPossible;
    }

    /**
     * Overloaded constructor for stationary or simple shots where exit speed
     * and shot possibility checks default.
     */
    public ShooterSolution(double flywheelRPM, double turretAngleDegrees, double hoodAngleDegrees) {
        this(flywheelRPM, turretAngleDegrees, hoodAngleDegrees, true);
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



    public boolean isShotPossible() {
        return isShotPossible;
    }

    @Override
    public String toString() {
        return String.format("ShooterSolution{RPM=%.1f, Turret=%.2f deg, Hood=%.2f deg, ReqSpeed=%.2f m/s, Possible=%b}",
                flywheelRPM, turretAngleDegrees, hoodAngleDegrees, isShotPossible);
    }
}