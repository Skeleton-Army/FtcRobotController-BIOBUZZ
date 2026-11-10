package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.ManualDrive;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.skeletonarmy.marrow.LynxUtil;
import com.skeletonarmy.marrow.telemetry.MarrowTelemetry;

import org.firstinspires.ftc.teamcode.R;
import org.firstinspires.ftc.teamcode.pedro.Constants;

@TeleOp(name="TeleOp")
public class TeleOpApp extends CommandOpMode {
    private DcMotorEx flywheel;
    private Follower follower;

    @Override
    public void initialize() {
        LynxUtil.setBulkCachingMode(hardwareMap, LynxModule.BulkCachingMode.MANUAL);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        follower = Constants.createPrimary(hardwareMap, PinpointLocalizer.ResetMode.NONE);
        flywheel = hardwareMap.get(DcMotorEx.class, "flywheel");
    }

    @Override
    public void run() {
        super.run();
        LynxUtil.clearBulkCache(hardwareMap);

        telemetry.addData("pos", flywheel.getCurrentPosition());
        ManualDrive.driveOrHold(
                follower,
                -gamepad1.left_stick_y,
                -gamepad1.left_stick_x,
                -gamepad1.right_stick_x
        );
        follower.update();
        telemetry.update();
    }
}
