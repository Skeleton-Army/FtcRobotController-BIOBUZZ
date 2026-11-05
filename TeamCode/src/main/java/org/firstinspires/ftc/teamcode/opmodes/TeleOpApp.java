package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;
import com.skeletonarmy.marrow.LynxUtil;

import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Intake;

import kotlin.time.Instant;

@TeleOp(name="TeleOp")
public class TeleOpApp extends CommandOpMode {
    private Follower follower;
    private Intake intake;
    private GamepadEx gamepad;
    private final Pose startPose = new Pose(0, 0, Math.toRadians(0));
    
    private boolean previousButtonB = false;
    private long lastPressButtonB = 0;
    private boolean isHoldingIntake = false;
    private boolean isToggledOn = false;

    private final int minThreshold = 150;
    private boolean currentButtonB;

    @Override
    public void initialize() {
        LynxUtil.setBulkCachingMode(hardwareMap, LynxModule.BulkCachingMode.MANUAL);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        follower = Constants.create(hardwareMap);
        follower.setPose(startPose);

        intake = new Intake(hardwareMap);

        gamepad = new GamepadEx(gamepad1);

        gamepad.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whenHeld(new InstantCommand(intake::releaseIntake));
        gamepad.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whenReleased(new InstantCommand(intake::stopIntake));
    }

    @Override
    public void run() {
        super.run();
        LynxUtil.clearBulkCache(hardwareMap);

        follower.manual(
                -gamepad1.left_stick_y,
                -gamepad1.left_stick_x,
                -gamepad1.right_stick_x
        );
        follower.update();

        currentButtonB = gamepad1.b;

        // 1. RISING EDGE: Button was just pressed down this loop cycle
        if (currentButtonB && !previousButtonB) {
            lastPressButtonB = System.currentTimeMillis();
            isHoldingIntake = false;
        }

        // 2. WHILE HELD: Button is currently being held down
        if (currentButtonB) {
            long passedTime = System.currentTimeMillis() - lastPressButtonB;

            // If held past your minThreshold, activate hold mode and start intake
            if (passedTime > minThreshold && !isHoldingIntake) {
                isHoldingIntake = true;
                schedule(new InstantCommand(intake::startIntake));
            }
        }
        // 3. FALLING EDGE: Button was just released this loop cycle
        else if (!currentButtonB && previousButtonB) {
            long passedTime = System.currentTimeMillis() - lastPressButtonB;

            // Short tap (under minThreshold): flip the toggle state
            if (passedTime <= minThreshold) {
                isToggledOn = !isToggledOn;
                if (isToggledOn) {
                    schedule(new InstantCommand(intake::startIntake));
                } else {
                    schedule(new InstantCommand(intake::stopIntake));
                }
            }
            // Long press release: stop intake (unless the toggle is keeping it running)
            else if (isHoldingIntake) {
                isHoldingIntake = false;
                if (!isToggledOn) {
                    schedule(new InstantCommand(intake::stopIntake));
                }
            }
        }

        // Update the previous button state for the next loop iteration
        previousButtonB = currentButtonB;


        telemetry.addData("location", follower.pose());
        telemetry.update();
    }
}
