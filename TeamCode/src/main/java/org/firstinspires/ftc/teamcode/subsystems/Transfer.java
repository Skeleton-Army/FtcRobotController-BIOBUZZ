package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.config.TransferConfig.TRANSFER_MOTOR;
import static org.firstinspires.ftc.teamcode.config.TransferConfig.TRANSFER_NAME;
import static org.firstinspires.ftc.teamcode.config.TransferConfig.TRANSFER_POWER;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;

public class Transfer extends SubsystemBase {
    private final MotorEx transfer;
    public Transfer (HardwareMap hardwareMap) {
        transfer = new MotorEx(hardwareMap, TRANSFER_NAME, TRANSFER_MOTOR);
    }

    public double getRPM() {
        double motorTPS = transfer.getCorrectedVelocity();
        return (motorTPS * 60.0) / transfer.getCPR();
    }

    public void startTransfer() {
        transfer.set(TRANSFER_POWER);
    }

    public void stopTransfer() {
        transfer.set(0);
    }

    public void reverseTransfer() {
        transfer.set(-TRANSFER_POWER);
    }
}
