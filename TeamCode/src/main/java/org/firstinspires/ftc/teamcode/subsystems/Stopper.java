package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.config.StopperConfig.RELEASE_POS;
import static org.firstinspires.ftc.teamcode.config.StopperConfig.BLOCK_POS;
import static org.firstinspires.ftc.teamcode.config.StopperConfig.STOPPER_NAME;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.hardware.servos.ServoEx;

public class Stopper extends SubsystemBase {
    private final ServoEx stopper;
    public Stopper(HardwareMap hardwareMap) {
        stopper = new ServoEx(hardwareMap, STOPPER_NAME);
        block();
    }

    public void block() {
        stopper.set(BLOCK_POS);
    }

    public void release() {
        stopper.set(RELEASE_POS);
    }

    public boolean isBlocking() {
        return stopper.get() == BLOCK_POS;
    }
}
