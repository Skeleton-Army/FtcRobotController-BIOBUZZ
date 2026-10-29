package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;
import com.skeletonarmy.marrow.TimerEx;

import org.firstinspires.ftc.teamcode.config.IntakeConfig;

@Config
public class Intake extends SubsystemBase {
    private final MotorEx intake;
    private boolean isCollecting = false;
    private final TimerEx runtime = new TimerEx();
    private double power;

    public Intake(final HardwareMap hardwareMap) {
        intake = new MotorEx(hardwareMap, IntakeConfig.INTAKE_NAME, IntakeConfig.INTAKE_MOTOR);
        intake.setInverted(true);
        runtime.start();
        runtime.pause();

        power = IntakeConfig.INTAKE_RELEASE_POWER;
    }

    public void startIntake() {
        if (!isCollecting) {
            runtime.restart();
        }
        intake.set(power);
        isCollecting = true;
    }

    public void setIntakeSpeed(double power) {
        this.power = power;
    }

    public void releaseIntake() {
        intake.set(-IntakeConfig.INTAKE_RELEASE_POWER);
        isCollecting = false;
    }

    public void stopIntake() {
        runtime.pause();
        intake.set(0);
        isCollecting = false;
    }

    public boolean isCollecting() {
        return isCollecting;
    }
}