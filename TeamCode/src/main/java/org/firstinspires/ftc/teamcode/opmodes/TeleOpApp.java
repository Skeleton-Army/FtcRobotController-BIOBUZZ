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
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.consts.HivePositions;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;

@TeleOp(name = "TeleOp")
public class TeleOpApp extends CommandOpMode {
    private Follower follower;
    private Shooter shooter;
    private double largestError = 0;

    @Override
    public void initialize() {
        LynxUtil.setBulkCachingMode(hardwareMap, LynxModule.BulkCachingMode.MANUAL);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        follower = Constants.createPrimary(hardwareMap, PinpointLocalizer.ResetMode.NONE);
        follower.setPose(new Pose(7, 7, Math.toRadians(0)));
        shooter = new Shooter(hardwareMap, follower);
    }

    @Override
    public void run() {
        super.run();
        LynxUtil.clearBulkCache(hardwareMap);

        ManualDrive.driveOrHold(follower, -gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x);

        double currentError = Math.abs(shooter.getTurret().getTargetAngle(AngleUnit.DEGREES)
                - shooter.getTurret().getAngle(AngleUnit.DEGREES));
        largestError = Math.max(largestError, currentError);
        if (gamepad1.a) largestError = 0;
        telemetry.addData("targetAngle", shooter.getTurret().getTargetAngle(AngleUnit.DEGREES));
        telemetry.addData("turretAngle", shooter.getTurret().getAngle(AngleUnit.DEGREES));
        telemetry.addData("currentError", currentError);
        telemetry.addData("largestError", largestError);
        shooter.setTarget(HivePositions.getHivePose(follower), Math.toRadians(35), 3000);
        follower.update();
        telemetry.update();
    }
}
