package org.firstinspires.ftc.teamcode.opmodes.utils;

import com.qualcomm.robotcore.eventloop.opmode.Utility;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;
import com.seattlesolvers.solverslib.hardware.servos.ServoEx;

import org.firstinspires.ftc.teamcode.config.HoodConfig;

@Utility
public class ResetHood extends CommandOpMode {
    private ServoEx hood;
    private GamepadEx gamepadEx;

    @Override
    public void initialize() {
        this.hood = new ServoEx(hardwareMap, "hood", HoodConfig.HOOD_POSSIBLE_MIN, HoodConfig.HOOD_POSSIBLE_MAX);
        this.gamepadEx = new GamepadEx(gamepad1);
        gamepadEx.getGamepadButton(GamepadKeys.Button.A)
                .whenActive(() -> hood.set(HoodConfig.HOOD_POSSIBLE_MIN));
    }

    @Override
    public void run() {
        super.run();
        telemetry.addLine("Press A or X to reset the hood");
        telemetry.addLine("Make sure to press it ONCE and not hold the button!");
        telemetry.addData("Hood Position", hood.get());
        telemetry.update();
    }
}
