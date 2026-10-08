package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.config.FlywheelConfig.RPM_REACHED_THRESHOLD;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;

public class Shooter extends SubsystemBase {
    private final Turret turret;
    private final Flywheel flywheel;
    private final Hood hood;

    public Shooter(final HardwareMap hardwareMap) {
        turret = new Turret(hardwareMap);
        flywheel = new Flywheel(hardwareMap);
        hood = new Hood(hardwareMap);
    }

    /**
     * @param turretAngleRad target turret angle in radians
     * @param hoodAngleRad   target hood angle in radians
     * @param rpm            target flywheel RPM
     */
    public void setTarget(double turretAngleRad, double hoodAngleRad, double rpm) {
        turret.setHorizontalAngle(turretAngleRad);
        hood.setVerticalAngle(hoodAngleRad);
        flywheel.setRPM(rpm);
    }

    public boolean flywheelAtTarget() {
        return Math.abs(flywheel.getTargetRPM() - flywheel.filteredRPM) <= RPM_REACHED_THRESHOLD;
    }

    public boolean readyToShoot() {
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
}