package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;

@Config
public class Flywheel extends SubsystemBase {
    private final MotorEx flywheel;
    public static double power = -1;
    public Flywheel(HardwareMap h) {
        this.flywheel = new MotorEx(h, "flywheel", Motor.GoBILDA.RPM_1150);
    }

    public void runFlywheel() {
        flywheel.set(power);
    }

    public void stopFlywheel() {
        flywheel.set(0);
    }
}
