package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.config.TurretConfig.*;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.controller.PIDController;
import com.seattlesolvers.solverslib.util.MathUtils;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class Turret extends SubsystemBase {
    private final DcMotorEx turret;
    private final PIDController turretPID;

    private final double distancePerPulse;

    private boolean disabled = false;
    private double wrapped;
    private double lastWrappedTarget = 0;
    private boolean hasWrappedTarget = false;

    public Turret(final HardwareMap hardwareMap) {
        turret = hardwareMap.get(DcMotorEx.class, TURRET_NAME);
        turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turret.setDirection(TURRET_INVERTED ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
        distancePerPulse = (Math.PI * 2) / (turret.getMotorType().getTicksPerRev() * GEAR_RATIO);

        turretPID = new PIDController(TURRET_KP, TURRET_KI, TURRET_KD);
        turretPID.setTolerance(TURRET_POSITION_TOLERANCE, TURRET_VELOCITY_TOLERANCE);

        setHorizontalAngle(0);
    }

    @Override
    public void periodic() {
        if (disabled) {
            turret.setPower(0);
            return;
        }

        double power = turretPID.calculate(getDistance());
        turret.setPower(Math.max(-1, Math.min(1, power)));
    }

    public void setHorizontalAngle(double targetAngleRad) {
        double lastCmd = hasWrappedTarget ? lastWrappedTarget : getDistance();

        wrapped = wrapToTarget(lastCmd, targetAngleRad, TURRET_MIN, TURRET_MAX, TURRET_WRAP, lastCmd);

        lastWrappedTarget = wrapped;
        hasWrappedTarget = true;

        turretPID.setSetPoint(wrapped);
    }

    public boolean reachedAngle() {
        return turretPID.atSetPoint();
    }

    public double getTurretAngle(AngleUnit angleUnit) {
        return angleUnit == AngleUnit.DEGREES ? Math.toDegrees(getDistance()) : getDistance();
    }

    /** Current turret position in radians. */
    private double getDistance() {
        return turret.getCurrentPosition() * distancePerPulse;
    }

    public void resetTurret() {
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void disable() {
        disabled = true;
    }

    public void enable() {
        disabled = false;
    }

    private double wrapToTarget(double current, double target, double min, double max, boolean wrap, double lastCommanded) {
        double delta = Math.atan2(Math.sin(target - current), Math.cos(target - current));
        double closest = current + delta;

        if (!wrap) return MathUtils.clamp(closest, min, max);

        boolean closestInRange = closest >= min && closest <= max;

        double wrapped = (closest < min) ? closest + 2 * Math.PI : closest - 2 * Math.PI;
        boolean wrappedInRange = wrapped >= min && wrapped <= max;

        // Only one side is valid -> unambiguous
        if (closestInRange && !wrappedInRange) return closest;
        if (!closestInRange && wrappedInRange) return wrapped;

        // From here on, only two cases remain: both valid (overlap band) or neither valid (deadzone).
        if (closestInRange) {
            // Overlap band: both sides are technically legal. Stick with
            // whichever is closer to what we last commanded, to avoid flip-flop.
            double distToClosest = Math.abs(closest - lastCommanded);
            double distToWrapped = Math.abs(wrapped - lastCommanded);
            return (distToClosest <= distToWrapped) ? closest : wrapped;
        }

        // Neither side reachable -> true deadzone, clamp to nearest limit
        return (closest < min) ? min : max;
    }
}