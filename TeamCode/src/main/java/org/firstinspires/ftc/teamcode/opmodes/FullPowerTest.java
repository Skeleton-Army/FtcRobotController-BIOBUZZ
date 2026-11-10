package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Utility;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;

@Utility
public class FullPowerTest extends CommandOpMode {
    private DcMotorEx motor1;
    private DcMotorEx motor2;
    private GamepadEx gamepad;

    @Override
    public void initialize() {
        motor1 = hardwareMap.get(DcMotorEx.class, "shooter1");
        motor2 = hardwareMap.get(DcMotorEx.class, "shooter2");

        gamepad = new GamepadEx(gamepad1);
        gamepad.getGamepadButton(GamepadKeys.Button.A)
                .whenActive(() -> {
                    motor1.setPower(1);
                    motor2.setPower(-1);
                })
                .whenInactive(() -> {
                   motor1.setPower(0);
                   motor2.setPower(0);
                });
    }
}
