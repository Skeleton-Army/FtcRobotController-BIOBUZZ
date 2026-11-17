package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;

public class Collector extends SubsystemBase {

    private final Intake intake;
    private final Transfer transfer;
    private final Stopper stopper;
    public Collector(HardwareMap hardwareMap){
        intake = new Intake(hardwareMap);
        transfer = new Transfer(hardwareMap);
        stopper = new Stopper(hardwareMap);
    }
    public void intake() {
        intake.startIntake();
        transfer.startTransfer();
        stopper.block();
    }

    public void feed() {
        transfer.startTransfer();
        stopper.release();
    }

    public void eject() {
        intake.reverseIntake();
        transfer.reverseTransfer();
        stopper.block();
    }

    public void stop() {
        intake.stopIntake();
        transfer.stopTransfer();
        stopper.block();
    }
}
