package org.firstinspires.ftc.teamcode.calculators;

import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;
import org.firstinspires.ftc.teamcode.config.ShooterConfig;
import org.firstinspires.ftc.teamcode.utilities.Polynomial;

/**
 * Polynomial-based implementation of {@link ShooterCalculator}.
 *
 * Given target geometry, robot motion, and current flywheel speed, it computes:
 * 1. The target flywheel RPM needed to score while compensating for robot motion.
 * 2. The turret relative angle needed to aim at the goal.
 * 3. The hood pitch angle needed to compensate for trajectory shifts.
 * 4. Required ball exit speed from the flywheel.
 * 5. Validity flags indicating whether the shot is physically achievable.
 */
public class BasicShooterCalculator implements ShooterCalculator {

    // ------------------------------------------------------------------
    // Models (all polynomials in ascending-degree order)
    // ------------------------------------------------------------------

    /** Distance to goal (m) -> hood pitch angle (degrees). Comes from the shooter config. */
    private final Polynomial hoodAngleModel;

    /** Distance to goal (m) -> ideal ball exit speed (m/s). */
    private final Polynomial exitSpeedModel;

    /**
     * Ball exit speed (m/s) -> flywheel RPM.
     * Must be linear (2 coefficients) so it can be inverted in {@link #rpmToExitSpeed(double)}.
     */
    private final Polynomial speedToRpmModel;

    /** Distance to goal (m) -> lowest exit speed (m/s) that still scores. */
    private final Polynomial minAllowedSpeedModel;

    /** Distance to goal (m) -> highest exit speed (m/s) that still scores. */
    private final Polynomial maxAllowedSpeedModel;

    // ------------------------------------------------------------------
    // Tolerances and turret geometry
    // ------------------------------------------------------------------

    /** Extra slack (m/s) allowed BELOW the minimum speed before a shot is called invalid. */
    private final double lowerSpeedBuffer;

    /** Extra slack (m/s) allowed ABOVE the maximum speed before a shot is called invalid. */
    private final double upperSpeedBuffer;

    /** Turret pivot offset from the robot center, along the robot's forward axis (m). */
    private final double turretOffsetForward;

    /** Turret pivot offset from the robot center, along the robot's left axis (m). */
    private final double turretOffsetLeft;

    /** Smallest launch pitch (radians) the hood can physically reach. From the config. */
    private final double hoodMinRadians;

    /** Largest launch pitch (radians) the hood can physically reach. From the config. */
    private final double hoodMaxRadians;

    public BasicShooterCalculator(ShooterConfig config,
                                  Polynomial exitSpeedModel,
                                  Polynomial speedToRpmModel,
                                  Polynomial minAllowedSpeedModel,
                                  Polynomial maxAllowedSpeedModel,
                                  double lowerSpeedBuffer,
                                  double upperSpeedBuffer,
                                  double turretOffsetForward,
                                  double turretOffsetLeft) {
        if (config == null) {
            throw new IllegalArgumentException("ShooterConfig cannot be null");
        }
        if (speedToRpmModel.getDegree() != 1) {
            throw new IllegalArgumentException(
                    "speedToRpmModel must be linear (2 coefficients) so RPM can be converted back to speed.");
        }
        this.hoodAngleModel = config.getHoodAngleModel();
        this.exitSpeedModel = exitSpeedModel;
        this.speedToRpmModel = speedToRpmModel;
        this.minAllowedSpeedModel = minAllowedSpeedModel;
        this.maxAllowedSpeedModel = maxAllowedSpeedModel;
        this.lowerSpeedBuffer = lowerSpeedBuffer;
        this.upperSpeedBuffer = upperSpeedBuffer;
        this.turretOffsetForward = turretOffsetForward;
        this.turretOffsetLeft = turretOffsetLeft;
        this.hoodMinRadians = Math.toRadians(config.getMinHoodAngle());
        this.hoodMaxRadians = Math.toRadians(config.getMaxHoodAngle());
    }

    // ------------------------------------------------------------------
    // ShooterCalculator interface
    // ------------------------------------------------------------------

    /**
     * Calculates the shot solution for a robot, compensating for robot velocity and current flywheel state.
     *
     * @param robotX             Robot X on the field (m)
     * @param robotY             Robot Y on the field (m)
     * @param robotHeading       Robot heading (radians)
     * @param goalX              Goal X on the field (m)
     * @param goalY              Goal Y on the field (m)
     * @param robotVelocityX     Robot field-frame velocity X (m/s)
     * @param robotVelocityY     Robot field-frame velocity Y (m/s)
     * @param currentFlywheelRpm Current measured flywheel RPM
     * @return Full solution containing flywheel RPM, turret angle, hood angle, exit speed, and shot validity.
     */
    @Override
    public ShooterSolution calculate(double robotX, double robotY, double robotHeading,
                                     double goalX, double goalY,
                                     double robotVelocityX, double robotVelocityY,
                                     double currentFlywheelRpm) {
        // Calculate turret pivot field position and distance/angle to goal
        double cosHeading = Math.cos(robotHeading);
        double sinHeading = Math.sin(robotHeading);

        double turretX = robotX + turretOffsetForward * cosHeading - turretOffsetLeft * sinHeading;
        double turretY = robotY + turretOffsetForward * sinHeading + turretOffsetLeft * cosHeading;

        double dx = goalX - turretX;
        double dy = goalY - turretY;
        double distance = Math.hypot(dx, dy);
        double angleToGoal = Math.atan2(dy, dx);

        Vector3D robotVelocity = new Vector3D(robotVelocityX, robotVelocityY, 0);

        // Step 1: Stationary ("ideal") launch velocity toward the goal
        double idealSpeed = calculateExitSpeed(distance);
        double hoodAngle = Math.toRadians(calculateHoodAngleDegrees(distance));
        Vector3D idealLaunch = buildLaunchVelocity(idealSpeed, hoodAngle, angleToGoal);

        // Step 2: Launch velocity the flywheel must supply, and the target RPM needed
        Vector3D requiredLaunch = idealLaunch.subtract(robotVelocity);
        double requiredSpeed = requiredLaunch.getNorm();
        int targetRpm = exitSpeedToRpm(requiredSpeed);

        // Step 3: What the flywheel is actually outputting right now
        double currentSpeed = rpmToExitSpeed(currentFlywheelRpm);
        Vector3D actualLaunch = buildLaunchVelocity(currentSpeed, hoodAngle, angleToGoal);

        // Vector direction to aim so that (launch - robot motion) hits the goal
        Vector3D aimVector = actualLaunch.subtract(robotVelocity);
        double aimTurretAngle = aimVector.getAlpha() - robotHeading; // relative to robot heading
        double aimHorizontalSpeed = Math.hypot(aimVector.getX(), aimVector.getY());
        double aimHoodAngle = Math.atan2(aimVector.getZ(), aimHorizontalSpeed);

        // Step 4: Physical bounds and validity checks
        boolean isHoodAngleReachable = aimHoodAngle >= hoodMinRadians && aimHoodAngle <= hoodMaxRadians;
        boolean isSpeedInCorridor = isExitSpeedValid(distance, currentSpeed, requiredSpeed);
        boolean isShotPossible = isHoodAngleReachable && isSpeedInCorridor;

        double finalTurretDegrees = Math.toDegrees(normalizeAngle(aimTurretAngle));
        double finalHoodDegrees = Math.toDegrees(aimHoodAngle);

        return new ShooterSolution(targetRpm, finalTurretDegrees, finalHoodDegrees, requiredSpeed, isShotPossible);
    }

    // ------------------------------------------------------------------
    // Calculations
    // ------------------------------------------------------------------

    public double calculateHoodAngleDegrees(double distance) {
        return hoodAngleModel.evaluate(distance);
    }

    public double calculateExitSpeed(double distance) {
        return exitSpeedModel.evaluate(distance);
    }

    public int exitSpeedToRpm(double exitSpeed) {
        return (int) speedToRpmModel.evaluate(exitSpeed);
    }

    public double rpmToExitSpeed(double rpm) {
        double[] coefficients = speedToRpmModel.getCoefficients();
        double intercept = coefficients[0];
        double slope = coefficients[1];
        return (rpm - intercept) / slope;
    }

    public boolean isExitSpeedValid(double distance, double exitSpeed) {
        return isExitSpeedValid(distance, exitSpeed, calculateExitSpeed(distance));
    }

    public boolean isExitSpeedValid(double distance, double exitSpeed, double targetSpeed) {
        double bandShift = targetSpeed - calculateExitSpeed(distance);

        double minSpeed = minAllowedSpeedModel.evaluate(distance) + bandShift;
        double maxSpeed = maxAllowedSpeedModel.evaluate(distance) + bandShift;

        return exitSpeed >= minSpeed - lowerSpeedBuffer
                && exitSpeed <= maxSpeed + upperSpeedBuffer;
    }

    public double calculateTurretAngleDegrees(double robotX, double robotY, double robotHeading,
                                              double goalX, double goalY) {
        double cosHeading = Math.cos(robotHeading);
        double sinHeading = Math.sin(robotHeading);

        double turretX = robotX + turretOffsetForward * cosHeading - turretOffsetLeft * sinHeading;
        double turretY = robotY + turretOffsetForward * sinHeading + turretOffsetLeft * cosHeading;

        double angleRadians = Math.atan2(goalY - turretY, goalX - turretX);
        return Math.toDegrees(angleRadians);
    }

    private Vector3D buildLaunchVelocity(double speed, double pitch, double headingAngle) {
        double horizontalSpeed = speed * Math.cos(pitch);
        return new Vector3D(
                horizontalSpeed * Math.cos(headingAngle),
                horizontalSpeed * Math.sin(headingAngle),
                speed * Math.sin(pitch));
    }

    private static double normalizeAngle(double angleRadians) {
        double angle = angleRadians % (2 * Math.PI);
        if (angle < 0) {
            return angle + 2 * Math.PI;
        }
        return angle;
    }
}