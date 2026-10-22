package org.firstinspires.ftc.teamcode.opmodes;


import static com.pedropathing.api.Paths.*;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.interpolator.Interpolator;
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
import org.firstinspires.ftc.teamcode.subsystems.Intake;

@Autonomous(name="Autonomous", preselectTeleOp="TeleOp")
public class AutonomousApp extends CommandOpMode {
    private final Prompter prompter = new Prompter(this);
    private Intake intake;
    private Alliance alliance;
    private Follower follower;
    private Path scorePathClose;
    private Path collectPath;
    private Path scorePathFar;
    private Path parkPath;

    private final Pose startPose = new Pose(63.3, 8, Math.toRadians(180));
    private final Pose scorePoseClose = new Pose(63.3, 15, Math.toRadians(180));
    private final Pose farControlPoint = new Pose(31, 52.5);
    private final Pose scorePoseFar = new Pose(34.5, 120, Math.toRadians(45));
    private final Pose collectPose = new Pose(12, 11, Math.toRadians(-103));
    private final Pose collectControlPoint = new Pose(30, 52, 0);
    private final Pose parkPose = new Pose(20, 119.6, Math.toRadians(45));

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
        ).heading(Interpolator.piecewise()
                .until(0.7, Interpolator.linear(changeAlliance(scorePoseClose, alliance), changeAlliance(collectPose, alliance)))
                .until(1, Interpolator.constant(changeAlliance(collectPose, alliance)))
        );
    }

    private Path getScorePathFar(Alliance alliance) {
        return curve(
                changeAlliance(collectPose, alliance),
                changeAlliance(farControlPoint, alliance),
                changeAlliance(scorePoseFar, alliance)
        ).constant(changeAlliance(collectPose, alliance));
    }

    private Path getParkPath(Alliance alliance) {
        return line(
                changeAlliance(scorePoseFar, alliance),
                changeAlliance(parkPose, alliance)
        ).constant(changeAlliance(collectPose , alliance));
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

        intake = new Intake(hardwareMap);

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
                        new InstantCommand(intake::startIntake), // turn on intake
                        new WaitCommand(2000),
                        new InstantCommand(intake::stopIntake), // turn off intake

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

