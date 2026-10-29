package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;
import com.seattlesolvers.solverslib.hardware.servos.ServoEx;

public class Hood extends SubsystemBase {
    private final ServoEx hood;
    public Hood(HardwareMap hardwareMap) {
        hood = new ServoEx(hardwareMap, "hood", .2, 1);

    }

    public void upbypointone() {
        hood.set(hood.getRawPosition() + .1);
    }

    public void downbypointone() {
        hood.set(hood.getRawPosition() - .1);
    }
}
