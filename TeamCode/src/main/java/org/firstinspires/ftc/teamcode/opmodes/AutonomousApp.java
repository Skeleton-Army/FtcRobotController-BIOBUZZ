package org.firstinspires.ftc.teamcode.opmodes;


import static com.pedropathing.api.Paths.line;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import android.graphics.Point;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.ScheduleCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.command.WaitUntilCommand;
import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand;
import com.skeletonarmy.marrow.LynxUtil;

import org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion;
import org.firstinspires.ftc.teamcode.pedro.Constants;

@Autonomous(name="Autonomous", preselectTeleOp="TeleOp")
public class AutonomousApp extends CommandOpMode {
    private Follower follower;
    private Path scorePath;
    private Path collectPath;
    private Path parkPath;
    private final Pose startPose   = new Pose(58.4795, 9.6037, Math.toRadians(90));
    private final Pose scorePose   = new Pose(6.5015, 105.8093, Math.toRadians(180));
    private final Pose collectPose = new Pose(10.1974, 8.1321, Math.toRadians(180));
    private final Pose parkPose    = new Pose(7.1278, 96.4563, Math.toRadians(180));


    @Override
        public void initialize() {
            LynxUtil.setBulkCachingMode(hardwareMap, LynxModule.BulkCachingMode.MANUAL);
            telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
            
        Follower follower = Constants.create(hardwareMap);
            follower.setPose(startPose);
            schedule(
                    new SequentialCommandGroup(
                            new FollowPathCommand(follower, scorePath),
                            new InstantCommand(), // Shoot
                            new WaitCommand(2000),
                            new FollowPathCommand(follower, parkPath)
            ));
            schedule(
                    new SequentialCommandGroup(
                            new FollowPathCommand(follower, collectPath),
                            new InstantCommand(), // turn on intake
                            new WaitCommand(1000),
                            new InstantCommand(), // turn off intake
                            new FollowPathCommand(follower, scorePath),
                            new InstantCommand(),// shoot
                            new WaitCommand(1500)
                            )
            );

        }

        @Override
        public void run() {
            super.run();
            LynxUtil.clearBulkCache(hardwareMap);

            follower.update();
        }
}

