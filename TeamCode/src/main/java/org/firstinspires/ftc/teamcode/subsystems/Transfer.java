package org.firstinspires.ftc.teamcode.subsystems;

import android.support.v4.app.INotificationSideChannel;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.hardware.HardwareDevice;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;
import com.seattlesolvers.solverslib.hardware.servos.ServoEx;

import org.firstinspires.ftc.teamcode.config.IntakeConfig;
import org.firstinspires.ftc.teamcode.config.TransferConfig;
import org.firstinspires.ftc.teamcode.consts.StopperStatus;

public class Transfer extends SubsystemBase {
    private final MotorEx transfer;
    private final ServoEx stopper;
    public Transfer(HardwareMap hardwareMap) {
        transfer = new MotorEx(hardwareMap, TransferConfig.TRANSFER_NAME, TransferConfig.TRANSFER_MOTOR);
        stopper = new ServoEx(hardwareMap, TransferConfig.STOPPERS_NAME, TransferConfig.STOPPER_MIN, TransferConfig.STOPPER_MAX);
    }

    public void block() {
        stopper.set(TransferConfig.STOPPER_MIN);
    }

    public void release() {
        stopper.set(TransferConfig.STOPPER_MAX);
    }

    public void startTransfer() {
        transfer.set(1);
    }

    public void stopTransfer() {
        transfer.set(0);
    }
}
