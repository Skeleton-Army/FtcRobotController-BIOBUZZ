package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.math.Pose;
import com.pedropathing.math.Velocity;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import java.util.function.Supplier;

import org.firstinspires.ftc.teamcode.calculators.ShooterCalculator;
import org.firstinspires.ftc.teamcode.calculators.ShooterSolution;
import org.firstinspires.ftc.teamcode.config.interfaces.ShooterConfig;
import org.firstinspires.ftc.teamcode.consts.HivePositions;

public class Shooter extends SubsystemBase {
    private final Turret turret;
    private final Flywheel flywheel;
    private final Hood hood;
    private final Supplier<Pose> poseSupplier;
    private final Supplier<Velocity> velocitySupplier;
    private final ShooterConfig config;
    private ShooterSolution solution;
    private final ShooterCalculator calculator;

    public Shooter(final HardwareMap hardwareMap, final Supplier<Pose> poseSupplier, Supplier<Velocity> velocitySupplier, ShooterConfig shooterConfig, ShooterCalculator shooterCalculator) {
        super();
        config = shooterConfig;
        calculator = shooterCalculator;
        this.poseSupplier = poseSupplier;
        this.velocitySupplier = velocitySupplier;

        turret = new Turret(hardwareMap, shooterConfig.getTurretConfig(), poseSupplier);
        flywheel = new Flywheel(hardwareMap, shooterConfig.getFlywheelConfig());
        hood = new Hood(hardwareMap, shooterConfig.getHoodConfig());
    }

    @Override
    public void periodic() {
        updateVoltage(VoltageGetter.getInstance().getVoltage());
    }

    /**
     * Set the turret and hood to a given point
     *
     * @param turretTargetPose target Pose
     * @param hoodAngleRad   target hood angle in radians
     * @param rpm            target flywheel RPM
     */
    public void setTarget(Pose turretTargetPose, double hoodAngleRad, double rpm) {
        turret.setTargetPose(turretTargetPose);
        hood.setVerticalAngle(hoodAngleRad);
        flywheel.setRPM(rpm);
    }

    public void setTarget(double hoodAngleRad, double rpm) {
        turret.setTargetPose(HivePositions.getHivePose(poseSupplier.get().y()));
        hood.setVerticalAngle(hoodAngleRad);
        flywheel.setRPM(rpm);
    }

    public boolean isReadyToShoot() {
        return turret.reachedAngle() && flywheel.isReadyToShoot() && hood.isAtTarget();
    }

    public void enable() {
        turret.enable();
        flywheel.enable();
    }

    public void disable() {
        turret.disable();
        flywheel.disable();
    }

    public Turret getTurret() {
        return turret;
    }

    public ShooterConfig getConfig() {
        return config;
    }

    public Flywheel getFlywheel() {
        return flywheel;
    }

    public Hood getHood() {
        return hood;
    }

    public void updateVoltage(double voltage) {
        this.flywheel.updateVoltage(voltage);
        this.turret.updateVoltage(voltage);
    }

    public void applySolution(ShooterSolution solution) {
        if (solution == null) {
            return;
        }

        flywheel.setRPM(solution.getFlywheelRPM());
        hood.setVerticalAngle(Math.toRadians(solution.getHoodAngleDegrees()));
        // why solution also turret?
    }

    public void updateTargetingByDistance(double distance) {
        if (calculator == null) {
            return;
        }

        double vx = 0.0;
        double vy = 0.0;

        Pose pose = poseSupplier.get();
        if (pose == null) {
            return;
        }


        Velocity velocity = velocitySupplier.get();
        if (velocity != null) {
            vx = velocity.vx;
            vy = velocity.vy;
        }

        double goalX = pose.x() + distance * Math.cos(pose.heading());
        double goalY = pose.y() + distance * Math.sin(pose.heading());
        double currentFlywheelRpm = flywheel.getRPM();
        Pose goalPose = new Pose(goalX, goalY);

        ShooterSolution solution = calculator.calculate(
                pose,
                goalPose,
                vx, vy,
                currentFlywheelRpm
        );
        applySolution(solution);
    }

    public void updateTargetingByPose(Pose targetPose) {
        if (targetPose != null && calculator != null) {
            Pose currentPose = poseSupplier.get();
            if (currentPose == null) {
                return;
            }

            Velocity velocity = velocitySupplier.get();
            double vx = velocity != null ? velocity.vx : 0.0;
            double vy = velocity != null ? velocity.vy : 0.0;
            double currentFlywheelRpm = flywheel.getRPM();

            ShooterSolution solution = calculator.calculate(
                    currentPose,
                    targetPose,
                    vx, vy,
                    currentFlywheelRpm
            );
            applySolution(solution);
        }
    }

    public void stop() {
        turret.disable();
        hood.home();
        flywheel.disable();
    }
}
