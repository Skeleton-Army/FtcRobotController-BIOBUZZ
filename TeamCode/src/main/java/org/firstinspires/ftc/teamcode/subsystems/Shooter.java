package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.teamcode.calculators.ShooterCalculator;
import org.firstinspires.ftc.teamcode.calculators.ShooterSolution;
import org.firstinspires.ftc.teamcode.config.ShooterConfig;

/**
 * Primary Shooter Subsystem for the solverslib architecture integrated with Pedro Pathing.
 * Aggregates child subsystem components (Turret, Flywheel, Hood) instantiated with a {@link ShooterConfig},
 * receives a Pedro {@link Follower}, and uses a {@link ShooterCalculator} to derive targeting solutions.
 */
public class Shooter extends SubsystemBase {

    private final Follower follower;
    private final Turret turret;
    private final Flywheel flywheel;
    private final Hood hood;
    private ShooterConfig config;
    private ShooterCalculator calculator;
    private ShooterSolution currentSolution;

    /**
     * Primary constructor taking a Pedro Follower, a ShooterConfig, and an optional ShooterCalculator.
     * Uses the ShooterConfig constants when instantiating the Turret, Flywheel, and Hood sub-objects.
     * Throws an {@link IllegalArgumentException} if config is null.
     *
     * @param follower Pedro Pathing Follower instance (can be null if unused)
     * @param config Base configuration containing subsystem parameters and constants (must not be null)
     * @param calculator Optional calculation strategy (if null, periodic will not run sub-components)
     */
    public Shooter(Follower follower, ShooterConfig config, ShooterCalculator calculator) {
        setName("ShooterMasterSubsystem");
        if (config == null) {
            throw new IllegalArgumentException("ShooterConfig cannot be null");
        }
        this.follower = follower;
        this.config = config;
        this.turret = new Turret(this.config);
        this.flywheel = new Flywheel(this.config);
        this.hood = new Hood(this.config);
        this.calculator = calculator;
    }

    /**
     * Overloaded constructor accepting a Pedro Follower and a ShooterConfig.
     *
     * @param follower Pedro Pathing Follower instance
     * @param config Base configuration containing subsystem parameters (must not be null)
     */
    public Shooter(Follower follower, ShooterConfig config) {
        this(follower, config, null);
    }

    @Override
    public void periodic() {
        // If calculator is null, periodic processing does not run sub-components
        if (calculator == null) {
            return;
        }
        turret.periodic();
        flywheel.periodic();
        hood.periodic();
    }

    // High-level Control Methods

    /**
     * Sets the flywheel target speed in RPM.
     *
     * @param rpm Target RPM
     */
    public void setFlywheelRPM(double rpm) {
        flywheel.setTargetRPM(rpm);
    }

    /**
     * Sets the target yaw angle for the turret in degrees.
     *
     * @param angleDegrees Target angle in degrees
     */
    public void setTurretAngle(double angleDegrees) {
        turret.setTargetAngle(angleDegrees);
    }

    /**
     * Sets the target pitch elevation angle for the hood in degrees.
     *
     * @param angleDegrees Target angle in degrees
     */
    public void setHoodAngle(double angleDegrees) {
        hood.setTargetAngle(angleDegrees);
    }

    /**
     * Applies a complete {@link ShooterSolution} to all child subsystems.
     *
     * @param solution Container holding RPM, turret angle, and hood angle
     */
    public void applySolution(ShooterSolution solution) {
        if (solution == null) return;
        this.currentSolution = solution;
        setFlywheelRPM(solution.getFlywheelRPM());
        setTurretAngle(solution.getTurretAngleDegrees());
        setHoodAngle(solution.getHoodAngleDegrees());
    }

    /**
     * Calculates and applies a shooter solution based on direct target distance.
     *
     * @param distance Distance to target in distance units
     */
    public void updateTargetingByDistance(double distance) {
        if (calculator != null) {
            ShooterSolution solution = calculator.calculate(distance);
            applySolution(solution);
        }
    }

    /**
     * Calculates distance from current Pedro Follower robot position to a target field Pose,
     * evaluates the solution using the calculator, and updates shooter targeting.
     *
     * @param targetPose Target coordinates in field frame
     */
    public void updateTargetingByPose(Pose targetPose) {
        if (follower != null && targetPose != null && calculator != null) {
            Pose currentPose = follower.pose();
            double dx = targetPose.x() - currentPose.x();
            double dy = targetPose.y() - currentPose.y();
            double distance = Math.hypot(dx, dy);
            ShooterSolution solution = calculator.calculate(distance);
            applySolution(solution);
        }
    }

    /**
     * Checks if all child subsystems (flywheel, turret, hood) are ready to shoot.
     *
     * @return True if flywheel speed, turret angle, and hood pitch match their target settings
     */
    public boolean isReadyToShoot() {
        return isFlywheelReady() && isTurretReady() && isHoodReady();
    }

    /**
     * Checks if the flywheel is at target speed.
     */
    public boolean isFlywheelReady() {
        return flywheel.isAtTargetSpeed();
    }

    /**
     * Checks if the turret is aligned at target angle.
     */
    public boolean isTurretReady() {
        return turret.isAtTargetAngle();
    }

    /**
     * Checks if the hood is set at target elevation pitch angle.
     */
    public boolean isHoodReady() {
        return hood.isAtTargetAngle();
    }

    /**
     * Stops shooter operation and resets components.
     */
    public void stop() {
        flywheel.stop();
        turret.homePosition();
        hood.retract();
    }

    // Component Accessors and Mutators

    public Follower getFollower() {
        return follower;
    }

    public Turret getTurret() {
        return turret;
    }

    public Flywheel getFlywheel() {
        return flywheel;
    }

    public Hood getHood() {
        return hood;
    }

    public ShooterConfig getConfig() {
        return config;
    }

    public void setConfig(ShooterConfig config) {
        if (config == null) {
            throw new IllegalArgumentException("ShooterConfig cannot be null");
        }
        this.config = config;
    }

    public ShooterCalculator getCalculator() {
        return calculator;
    }

    public void setCalculator(ShooterCalculator calculator) {
        this.calculator = calculator;
    }

    public ShooterSolution getCurrentSolution() {
        return currentSolution;
    }
}
