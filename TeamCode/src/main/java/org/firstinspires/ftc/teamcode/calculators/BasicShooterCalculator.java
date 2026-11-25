package org.firstinspires.ftc.teamcode.calculators;

import com.pedropathing.math.Pose;
import com.skeletonarmy.marrow.OpModeManager;
import org.firstinspires.ftc.teamcode.config.interfaces.ShooterConfig;
import org.firstinspires.ftc.teamcode.utils.Polynomial;

/**
 * Direct polynomial-based implementation of {@link ShooterCalculator} without
 * exit-speed velocity conversions or speed-corridor range checks, while retaining
 * physical hood angle limit verification.
 */
public class BasicShooterCalculator implements ShooterCalculator {

    // ------------------------------------------------------------------
    // Models (polynomials in ascending-degree order)
    // ------------------------------------------------------------------

    /** Distance to goal (m) -> target flywheel RPM. Comes from the shooter config. */
    private final Polynomial flywheelRpmModel;

    /** Distance to goal (m) -> hood pitch angle (degrees). Comes from the shooter config. */
    private final Polynomial hoodAngleModel;

    // ------------------------------------------------------------------
    // Turret geometry and hood limits
    // ------------------------------------------------------------------

    /** Turret pivot offset from the robot center, along the robot's forward axis (m). */
    private final double turretOffsetForward;

    /** Turret pivot offset from the robot center, along the robot's left axis (m). */
    private final double turretOffsetLeft;

    /** Smallest launch pitch (degrees) the hood can physically reach. From the config. */
    private final double minHoodAngleDeg;

    /** Largest launch pitch (degrees) the hood can physically reach. From the config. */
    private final double maxHoodAngleDeg;

    private static final double INCH_TO_METER = 0.0254;
    private static final double MPS_TO_RPM = 265.0;

    public BasicShooterCalculator(ShooterConfig config, double turretOffsetForward, double turretOffsetLeft) {
        if (config == null) {
            throw new IllegalArgumentException("ShooterConfig cannot be null");
        }
        this.flywheelRpmModel = config.getFlywheelConfig().getModel();
        this.hoodAngleModel = config.getHoodConfig().getModel();
        this.turretOffsetForward = turretOffsetForward;
        this.turretOffsetLeft = turretOffsetLeft;
        this.minHoodAngleDeg = Math.toDegrees(config.getHoodConfig().getHoodMin());
        this.maxHoodAngleDeg = Math.toDegrees(config.getHoodConfig().getHoodMax());
    }

    // ------------------------------------------------------------------
    // ShooterCalculator interface
    // ------------------------------------------------------------------

    @Override
    public ShooterSolution calculate(
            Pose robotPose, Pose goalPose, double robotVelocityX, double robotVelocityY, double currentFlywheelRpm) {
        double cosHeading = Math.cos(robotPose.heading());
        double sinHeading = Math.sin(robotPose.heading());

        // Turret pivot position in field coordinates
        double turretX = robotPose.x() + turretOffsetForward * cosHeading - turretOffsetLeft * sinHeading;
        double turretY = robotPose.y() + turretOffsetForward * sinHeading + turretOffsetLeft * cosHeading;

        double dx = goalPose.x() - turretX;
        double dy = goalPose.y() - turretY;
        double distance = Math.hypot(dx, dy);

        OpModeManager.getTelemetry().addData("goal x", goalPose.x());
        OpModeManager.getTelemetry().addData("goal y", goalPose.y());
        OpModeManager.getTelemetry().addData("turret x", turretX);
        OpModeManager.getTelemetry().addData("turret y", turretY);
        OpModeManager.getTelemetry().addData("distance", distance);
        double fieldAngleToGoal = Math.atan2(dy, dx);

        // Direct polynomial outputs from distance

        double distanceM = distance * INCH_TO_METER;
        int targetRpm = (int) Math.round(flywheelRpmModel.evaluate(distanceM) * MPS_TO_RPM);
        double hoodAngleDeg = hoodAngleModel.evaluate(distanceM);

        // Project robot velocity onto axes perpendicular and parallel to the goal line
        // to adjust turret lead angle without requiring an exit speed variable
        double dirX = distance > 1e-6 ? dx / distance : cosHeading;
        double dirY = distance > 1e-6 ? dy / distance : sinHeading;
        double perpX = -dirY;
        double perpY = dirX;

        double crossVelocity = robotVelocityX * perpX + robotVelocityY * perpY;
        double radialVelocity = robotVelocityX * dirX + robotVelocityY * dirY;

        // Small angular lead compensation based on lateral motion relative to distance
        double angularLeadRad = (distance > 1e-6) ? Math.atan2(-crossVelocity * 0.15, Math.max(distance, 0.25)) : 0.0;

        // Slight RPM adjustment for radial approach/recession
        int compensatedRpm = (int) Math.round(targetRpm - radialVelocity * 25.0);

        double relativeTurretRad = normalizeAngle((fieldAngleToGoal + angularLeadRad) - robotPose.heading());
        double finalTurretDegrees = Math.toDegrees(relativeTurretRad);
        OpModeManager.getTelemetry().addData("hood Angle (calc)", hoodAngleDeg);
        // Keep physical hood bounds check
        boolean isHoodAngleReachable = isHoodAngleValid(hoodAngleDeg);
        OpModeManager.getTelemetry().addData("isHoodAngleValid", isHoodAngleReachable);

        return new ShooterSolution(compensatedRpm, finalTurretDegrees, hoodAngleDeg, isHoodAngleReachable);
    }

    // ------------------------------------------------------------------
    // Calculations
    // ------------------------------------------------------------------

    public int calculateFlywheelRpm(double distance) {
        return (int) Math.round(flywheelRpmModel.evaluate(distance));
    }

    public double calculateHoodAngleDegrees(double distance) {
        OpModeManager.getTelemetry().addData("hood Angle (calc hood in calc)", hoodAngleModel.evaluate(distance));
        return hoodAngleModel.evaluate(distance);
    }

    public boolean isHoodAngleValid(double hoodAngleDegrees) {
        return hoodAngleDegrees >= minHoodAngleDeg && hoodAngleDegrees <= maxHoodAngleDeg;
    }

    public double calculateTurretAngleDegrees(
            double robotX, double robotY, double robotHeading, double goalX, double goalY) {
        double cosHeading = Math.cos(robotHeading);
        double sinHeading = Math.sin(robotHeading);

        double turretX = robotX + turretOffsetForward * cosHeading - turretOffsetLeft * sinHeading;
        double turretY = robotY + turretOffsetForward * sinHeading + turretOffsetLeft * cosHeading;

        double angleRadians = Math.atan2(goalY - turretY, goalX - turretX);
        return Math.toDegrees(angleRadians);
    }

    private static double normalizeAngle(double angleRadians) {
        double angle = angleRadians % (2.0 * Math.PI);
        if (angle < 0.0) {
            angle += 2.0 * Math.PI;
        }
        return angle;
    }
}
