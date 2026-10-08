package org.firstinspires.ftc.teamcode.opmodes;


import static com.pedropathing.api.Paths.*;

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
import com.skeletonarmy.marrow.prompts.OptionPrompt;
import com.skeletonarmy.marrow.prompts.Prompter;

import org.firstinspires.ftc.teamcode.enums.Alliance;
import org.firstinspires.ftc.teamcode.pedro.Constants;

@Autonomous(name="Autonomous", preselectTeleOp="TeleOp")
public class AutonomousApp extends CommandOpMode {
    private final Prompter prompter = new Prompter(this);
    private Alliance alliance;
    private Follower follower;
    private Path scorePathClose;
    private Path collectPath;
    private Path scorePathFar;
    private Path parkPath;

    private final Pose startPose = new Pose(55, 8, Math.toRadians(180));
    private final Pose scorePoseClose = new Pose(58.5, 45.2417, Math.toRadians(180));
    private final Pose scorePoseFar = new Pose(34.583, 119.1596, Math.toRadians(45));
    private final Pose collectPose = new Pose(10.5, 9.2, Math.toRadians(-103));
    private final Pose collectControlPoint = new Pose(20, 52, 0);
    private final Pose parkPose = new Pose(9.8, 119.6, Math.toRadians(45));
    //private final Pose farControlPoint1 = new Pose(5.0187, 41.4496, Math.toRadians(0));
    //private final Pose farControlPoint2 = new Pose(30.5131, 122.5821, Math.toRadians(0));

    public static Pose mirrorPose(Pose pose) {
        double newX = 144 - pose.x();
        double newY = 144 -pose.y();
        double newHeading = Math.toRadians(180) - pose.heading();

        return new Pose(newX, newY, newHeading);
    }

    public static Pose changeAlliance(Pose pose, Alliance alliance) {
        if (alliance == Alliance.BLUE) {
            pose = mirrorPose(pose);
        }

        return pose;
    }
    private Path getScorePathClose(Alliance alliance) {
        return line(
                changeAlliance(startPose, alliance),
                changeAlliance(scorePoseClose, alliance)
        ).constant(changeAlliance(startPose, alliance));
    }

    private Path getCollectPath(Alliance alliance) {
        return curve(
                changeAlliance(scorePoseClose, alliance),
                changeAlliance(collectControlPoint, alliance),
                changeAlliance(collectPose, alliance)
        ).linear(changeAlliance(scorePoseClose, alliance), changeAlliance(collectPose, alliance));
    }

    private Path getScorePathFar(Alliance alliance) {
        return line(
                changeAlliance(collectPose, alliance),
                changeAlliance(scorePoseFar, alliance)
        ).linear(changeAlliance(collectPose, alliance), changeAlliance(scorePoseFar, alliance));
    }

    private Path getParkPath(Alliance alliance) {
        return line(
                changeAlliance(scorePoseFar, alliance),
                changeAlliance(parkPose, alliance)
        ).constant(changeAlliance(scorePoseFar , alliance));
    }

    @Override
    public void initialize() {
        prompter.prompt("alliance", new OptionPrompt<>("SELECT ALLIANCE", Alliance.RED, Alliance.BLUE))
                .onComplete(() -> alliance = prompter.getOrDefault("alliance", Alliance.RED)
        );


        LynxUtil.setBulkCachingMode(hardwareMap, LynxModule.BulkCachingMode.MANUAL);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        follower = Constants.create(hardwareMap);
        follower.setPose(changeAlliance(startPose, alliance));

        scorePathClose = getScorePathClose(alliance);
        collectPath = getCollectPath(alliance);
        scorePathFar = getScorePathFar(alliance);
        parkPath = getParkPath(alliance);
        schedule(
                new SequentialCommandGroup(
                        new FollowPathCommand(follower, scorePathClose),
                        new InstantCommand(), // Shoot
                        new WaitCommand(2000),

                        new FollowPathCommand(follower, collectPath),
//                        new InstantCommand(), // turn on intake
                        new WaitCommand(2000),
//                        new InstantCommand(), // turn off intake

                        new FollowPathCommand(follower, scorePathFar),
                        new InstantCommand(), // shoot
                        new WaitCommand(1500),

                        new FollowPathCommand(follower, parkPath),
                        new WaitCommand(1000)
                )
        );
    }

    @Override
    public void initialize_loop() {
        prompter.run();
    }

    @Override
    public void run() {
        super.run();
        LynxUtil.clearBulkCache(hardwareMap);

        follower.update();

        Pose p = follower.pose();
        telemetry.addData("X", p.x());
        telemetry.addData("Y", p.y());
        telemetry.addData("Heading (deg)", Math.toDegrees(p.heading()));
        telemetry.update();
    }
}

