package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.ManualDrive;
import com.pedropathing.math.Pose;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.skeletonarmy.marrow.LynxUtil;
import com.skeletonarmy.marrow.telemetry.MarrowTelemetry;

import org.firstinspires.ftc.teamcode.R;
import org.firstinspires.ftc.teamcode.calculators.BasicShooterCalculator;
import org.firstinspires.ftc.teamcode.config.PollenShooterConfig;
import org.firstinspires.ftc.teamcode.config.ShooterConfig;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;

@TeleOp(name="TeleOp")
public class TeleOpApp extends CommandOpMode {
    private Follower follower;
    private Shooter shooter;
    private PollenShooterConfig PollenShooterConfig;


    @Override
    public void initialize() {
        LynxUtil.setBulkCachingMode(hardwareMap, LynxModule.BulkCachingMode.MANUAL);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        follower = Constants.createPrimary(hardwareMap, PinpointLocalizer.ResetMode.NONE);
        PollenShooterConfig = new PollenShooterConfig();
        shooter = new Shooter(follower, PollenShooterConfig, new BasicShooterCalculator(PollenShooterConfig, 1, 1));
        //todo: find actual turret offsets and replace
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
        shooter.updateTargetingByPose(new Pose(57.24, 56.13));
        telemetry.addData("flywheel rpm", shooter.getFlywheel().getTargetRPM());
        telemetry.addData("turret angle", shooter.getTurret().getTargetAngle());
        telemetry.addData("hood angle", shooter.getHood().getTargetAngle());
        telemetry.addData("polynomial manual", PollenShooterConfig.getHoodAngleModel().evaluate(150));
        telemetry.update();
    }
}
