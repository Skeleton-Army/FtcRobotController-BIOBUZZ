package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.ManualDrive;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;
import com.skeletonarmy.marrow.LynxUtil;

import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Flywheel;
import org.firstinspires.ftc.teamcode.subsystems.Hood;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Transfer;

@TeleOp(name="The Dark Day\uD83D\uDE36")
public class TeleOpApp extends CommandOpMode {
    private Follower follower;
    private Transfer transfer;
    private Intake intake;
    private Flywheel flywheel;
    private Hood kingvon;
    private GamepadEx gamepad;

    @Override
    public void initialize() {
        LynxUtil.setBulkCachingMode(hardwareMap, LynxModule.BulkCachingMode.MANUAL);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        follower = Constants.createWithoutReset(hardwareMap);
        intake = new Intake(hardwareMap);
        transfer = new Transfer(hardwareMap);
        flywheel = new Flywheel(hardwareMap);
        kingvon = new Hood(hardwareMap);
        gamepad = new GamepadEx(gamepad1);

        gamepad.getGamepadButton(GamepadKeys.Button.A)
                .whenActive(runIntake())
                .whenInactive(stopIntake());

        gamepad.getGamepadButton(GamepadKeys.Button.Y)
                .toggleWhenActive(
                        new InstantCommand(flywheel::runFlywheel),
                        new InstantCommand(flywheel::stopFlywheel)
                );

        gamepad.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .whenActive(shoot())
                .whenInactive(noShoot());

        gamepad.getGamepadButton(GamepadKeys.Button.DPAD_UP)
                .whenActive(upHood());

        gamepad.getGamepadButton(GamepadKeys.Button.DPAD_DOWN)
                .whenActive(downHood());
    }

    @Override
    public void run() {
        super.run();
        LynxUtil.clearBulkCache(hardwareMap);

        ManualDrive.driveOrHold(
                follower,
                -gamepad1.left_stick_y,
                -gamepad1.left_stick_x,
                -gamepad1.right_stick_x
        );
        follower.update();
    }

    private Command runIntake() {
        return new ParallelCommandGroup(
                new InstantCommand(intake::startIntake),
                new InstantCommand(transfer::startTransfer)
        );
    }

    private Command stopIntake() {
        return new ParallelCommandGroup(
                new InstantCommand(intake::stopIntake),
                new InstantCommand(transfer::stopTransfer)
        );
    }

    private Command shoot() {
        return new ParallelCommandGroup(
                new InstantCommand(transfer::release),
                new InstantCommand(transfer::startTransfer)
        );
    }

    private Command noShoot() {
        return new SequentialCommandGroup(
               new InstantCommand(transfer::block),
               new InstantCommand(transfer::stopTransfer)
        );
    }

    private Command upHood() {
        return new InstantCommand(kingvon::upbypointone);
    }

    private Command downHood() {
        return new InstantCommand(kingvon::downbypointone);
    }
}
