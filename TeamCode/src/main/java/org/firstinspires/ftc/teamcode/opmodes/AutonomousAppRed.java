package org.firstinspires.ftc.teamcode.opmodes;


import static com.pedropathing.api.Paths.curve;
import static com.pedropathing.api.Paths.line;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
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


@Autonomous(name="Autonomous", preselectTeleOp="TeleOp")
public class AutonomousAppRed extends CommandOpMode {
    private Follower follower;
    private Path scorePathClose;
    private Path collectPath;
    private Path parkPath;
    private Path scorePath;
    private final Pose startPose   = new Pose(58.4795, 9.6037, Math.toRadians(90));
    private final Pose scorePoseClose   = new Pose(6.5015, 105.8093, Math.toRadians(180));
    private final Pose collectPose = new Pose(10.1974, 8.1321, Math.toRadians(180));
    private final Pose parkPose    = new Pose(7.1278, 96.4563, Math.toRadians(180));
    private final Pose getScorePoseClose = new Pose(34.583, 119.1596, -31.0901);
    private final Pose point1Control1 = new Pose(5.0187, 41.4496, 0);
    private final Pose point1Control2 = new Pose(30.5131, 122.5821, 0);

    private Path getScorePathFar() {
        return curve(startPose, point1Control1, point1Control2).linear(startPose, point1Control2);
    }

    @Override
        public void initialize() {
            LynxUtil.setBulkCachingMode(hardwareMap, LynxModule.BulkCachingMode.MANUAL);
            telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        Follower follower = Constants.create(hardwareMap);
            follower.setPose(startPose);
            schedule(
                    new SequentialCommandGroup(
                            new FollowPathCommand(follower, scorePathClose),
                            new InstantCommand(), // Shoot
                            new WaitCommand(2000),

                            new FollowPathCommand(follower, collectPath),
                            new InstantCommand(), // turn on intake
                            new WaitCommand(1000),
                            new InstantCommand(), // turn off intake
                            new FollowPathCommand(follower, scorePathClose),
                            new InstantCommand(),// shoot
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

