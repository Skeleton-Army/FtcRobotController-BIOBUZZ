package org.firstinspires.ftc.teamcode.opmodes;


import static com.pedropathing.api.Paths.*;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand;
import com.skeletonarmy.marrow.LynxUtil;

import org.firstinspires.ftc.teamcode.pedro.Constants;

@Autonomous(name="AutonomousRed", preselectTeleOp="TeleOp")
public class AutonomousAppRed extends CommandOpMode {
    private Follower follower;

    private Path scorePathClose;
    private Path collectPath;
    private Path scorePathFar;
    private Path parkPath;

    private final Pose startPose = new Pose(58.4795, 9.6037, Math.toRadians(90));
    private final Pose scorePoseClose = new Pose(58.244, 45.2417, Math.toRadians(90.3786));
    private final Pose scorePoseFar = new Pose(34.583, 119.1596, Math.toRadians(-31.0901));
    private final Pose collectPose = new Pose(9.6694, 6.5481, Math.toRadians(45));
    private final Pose parkPose = new Pose(6.0718, 97.5123, Math.toRadians(120.8018));
    private final Pose farControlPoint1 = new Pose(5.0187, 41.4496, Math.toRadians(0));
    private final Pose farControlPoint2 = new Pose(30.5131, 122.5821, Math.toRadians(0));
    private Path getScorePathClose() {
        return line(startPose, scorePoseClose).linear(startPose, scorePoseClose);
    }
    private Path getCollectPath() {
        return line(scorePoseClose, collectPose).linear(scorePoseClose, collectPose);
    }
    private Path getScorePathFar() {
        return curve(collectPose, farControlPoint1, farControlPoint2, scorePoseFar).linear(collectPose, scorePoseFar);
    }
    private Path getParkPath() {
        return line(scorePoseFar, parkPose).linear(scorePoseFar, parkPose);
    }

    @Override
    public void initialize() {
        LynxUtil.setBulkCachingMode(hardwareMap, LynxModule.BulkCachingMode.MANUAL);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        follower = Constants.create(hardwareMap);
        follower.setPose(startPose);

        scorePathClose = getScorePathClose();
        collectPath = getCollectPath();
        scorePathFar = getScorePathFar();
        parkPath = getParkPath();
        schedule(
                new SequentialCommandGroup(
                        new FollowPathCommand(follower, scorePathClose),
                        new InstantCommand(), // Shoot
                        new WaitCommand(2000),

                        new FollowPathCommand(follower, collectPath),
                        new InstantCommand(), // turn on intake
                        new WaitCommand(1000),
                        new InstantCommand(), // turn off intake

                        new FollowPathCommand(follower, scorePathFar),
                        new InstantCommand(), // shoot
                        new WaitCommand(1500),

                        new FollowPathCommand(follower, parkPath)
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

