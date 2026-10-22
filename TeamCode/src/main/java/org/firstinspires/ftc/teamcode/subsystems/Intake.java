package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.config.IntakeConfig.*;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;
import com.skeletonarmy.marrow.TimerEx;

public class Intake extends SubsystemBase {
    private final MotorEx intake;
    private final MotorEx transfer;
    private boolean isCollecting = false;
    private final TimerEx runtime = new TimerEx();

    private double power;

    public Intake(final HardwareMap hardwareMap) {
        intake = new MotorEx(hardwareMap, INTAKE_NAME, INTAKE_MOTOR);
        intake.setInverted(true);
        transfer = new MotorEx(hardwareMap, TRANSFER_NAME, TRANSFER_MOTOR);
        runtime.start();
        runtime.pause();

        power = INTAKE_RELEASE_POWER;
    }

    public double getRPM() {
        double motorTPS = intake.getCorrectedVelocity();
        return (motorTPS * 60.0) / intake.getCPR();
    }

    public void startIntake() {
        if (!isCollecting) {
            runtime.restart();
        }
        intake.set(power);
        transfer.set(1);
        isCollecting = true;
    }

    public void setIntakeSpeed(double power) {
        this.power = power;
    }

    public void releaseIntake() {
        intake.set(-INTAKE_RELEASE_POWER);
        isCollecting = false;
    }

    public void stopIntake() {
        runtime.pause();
        intake.set(0);
        transfer.set(0);
        isCollecting = false;
    }

    public boolean isCollecting() {
        return isCollecting;
    }
}
