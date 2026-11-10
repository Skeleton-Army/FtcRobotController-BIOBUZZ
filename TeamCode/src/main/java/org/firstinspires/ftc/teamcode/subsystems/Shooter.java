package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.config.FlywheelConfig.RPM_REACHED_THRESHOLD;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import org.firstinspires.ftc.teamcode.consts.HivePositions;

public class Shooter extends SubsystemBase {
    private final Turret turret;
    private final Flywheel flywheel;
    private final Hood hood;
    private final Follower follower;

    public Shooter(final HardwareMap hardwareMap, final Follower follower) {
        super();

        this.follower = follower;
        turret = new Turret(hardwareMap, follower);
        flywheel = new Flywheel(hardwareMap);
        hood = new Hood(hardwareMap);
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
        turret.setTargetPose(HivePositions.getHivePose(follower));
        hood.setVerticalAngle(hoodAngleRad);
        flywheel.setRPM(rpm);
    }

    @Override
    public void periodic() {
        updateVoltage(VoltageGetter.getInstance().getVoltage());
    }

    public boolean flywheelAtTarget() {
        return Math.abs(flywheel.getTargetRPM() - flywheel.filteredRPM) <= RPM_REACHED_THRESHOLD;
    }

    public boolean isReadyToShoot() {
        return turret.reachedAngle() && flywheelAtTarget();
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
}
