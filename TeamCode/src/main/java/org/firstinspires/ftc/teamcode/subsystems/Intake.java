package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.config.IntakeConfig.*;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;

public class Intake extends SubsystemBase {
    private final MotorEx intake;

    public Intake(final HardwareMap hardwareMap) {
        intake = new MotorEx(hardwareMap, INTAKE_NAME, INTAKE_MOTOR);
        intake.setInverted(true);
    }

    public double getRPM() {
        double motorTPS = intake.getCorrectedVelocity();
        return (motorTPS * 60.0) / intake.getCPR();
    }

    public void startIntake() {
        intake.set(INTAKE_POWER);
    }

    public void reverseIntake() {
        intake.set(-INTAKE_POWER);
    }

    public void stopIntake() {
        intake.set(0);
    }
}
