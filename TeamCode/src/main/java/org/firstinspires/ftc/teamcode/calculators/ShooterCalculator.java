package org.firstinspires.ftc.teamcode.calculators;

/**
 * Interface defining calculation strategies for shooter target solutions.
 * Provides abstract calculation signatures for mapping spatial inputs (e.g. distance, orientation)
 * to output subsystem target parameters.
 */
public interface ShooterCalculator {

    /**
     * Calculates the target shooter solution given a distance parameter.
     *
     * @return Calculated {@link ShooterSolution} holding RPM, turret angle, and hood angle
     */
    ShooterSolution calculate(double robotX, double robotY, double robotHeading,
                              double goalX, double goalY,
                              double robotVelocityX, double robotVelocityY,
                              double currentFlywheelRpm);
}
