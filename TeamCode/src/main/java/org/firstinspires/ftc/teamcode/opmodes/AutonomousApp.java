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
import org.firstinspires.ftc.teamcode.enums.Parking;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Intake;

@Autonomous(name="Autonomous", preselectTeleOp="TeleOp")
public class AutonomousApp extends CommandOpMode {
    private final Prompter prompter = new Prompter(this);
    private Intake intake;
    private Alliance alliance;
    private Parking parking;
    private Follower follower;
    private Path scorePathClose;
    private Path collectPath;
    private Path scorePathFar;
    private Path parkPath;

    private final Pose startPose = new Pose(63.3, 8, Math.toRadians(180));
    private final Pose scorePoseClose = new Pose(63.3, 15, Math.toRadians(180));
    private final Pose scoreCloseControlPoint = new Pose(35, 52.5);
    private final Pose scoreFarControlPoint = new Pose(28.5, 130);
    private final Pose scorePoseFar = new Pose(48.5, 120, Math.toRadians(45));
    private final Pose collectPose = new Pose(15, 10, Math.toRadians(-103));
    private final Pose collectControlPoint = new Pose(30, 52, 0);
    private final Pose parkPoseCornerSide = new Pose(24, 118, Math.toRadians(270));
    private final Pose parkPoseFlowerSide = new Pose(24, 92, Math.toRadians(270));

    public void afterPromptsInitialize() {
        parking = prompter.getOrDefault("parking", Parking.CORNER_SIDE);
        alliance = prompter.getOrDefault("alliance", Alliance.RED);
        follower.setPose(startPose);
        scorePathClose = getScorePathClose(alliance);
        collectPath = getCollectPath(alliance);
        scorePathFar = getScorePathFar(alliance);
        parkPath = getParkPath(alliance, parking);
        afterPrompts();
    }
    public void afterPrompts() {
        schedule(
                new SequentialCommandGroup(
                        new FollowPathCommand(follower, scorePathClose),
                        new InstantCommand(), // Shoot
                        new WaitCommand(2000),

                        new FollowPathCommand(follower, collectPath),
                        new InstantCommand(intake::startTransfer),
                        pulseIntake(4),
                        new InstantCommand(intake::stopTransfer),

                        new FollowPathCommand(follower, scorePathFar),
                        new InstantCommand(), // shoot
                        new WaitCommand(1500),

                        new FollowPathCommand(follower, parkPath),
                        new WaitCommand(1000)
                )
        );
    }
        public static Pose mirrorPose(Pose pose) {
        double newX = 144 - pose.x();
        double newY = 144 -pose.y();
        double newHeading = Math.toRadians(180) - pose.heading();

        return new Pose(newX, newY, newHeading);
    }

    private Path getScorePathClose(Alliance alliance) {
        return line(
                startPose,
                scorePoseClose
        ).constant(startPose);
    }

    private Path getCollectPath(Alliance alliance) {
        return curve(
                scorePoseClose,
                collectControlPoint,
                collectPose
        ).heading(Interpolator.piecewise()
                .until(0.7, Interpolator.linear(scorePoseClose, collectPose))
                .until(1, Interpolator.constant(collectPose))
        );
    }

    private Path getScorePathFar(Alliance alliance) {
        return curve(
                collectPose,
                scoreCloseControlPoint,
                scoreFarControlPoint,
                scorePoseFar
        ).constant(collectPose);
    }

    private Path getParkPath(Alliance alliance, Parking parking) {
        if (parking == Parking.CORNER_SIDE) {
            return line(
                    scorePoseFar,
                    parkPoseCornerSide
            ).linear(collectPose , parkPoseCornerSide);
        }
        else if (parking == Parking.FLOWER_SIDE){
            return line(
                    scorePoseFar,
                    parkPoseFlowerSide
            ).linear(collectPose, parkPoseFlowerSide);
        }
        return null;
    }

    private SequentialCommandGroup pulseIntake(int pulses) {
        SequentialCommandGroup group = new SequentialCommandGroup();
        for (int i = 0; i < pulses; i++) {
        group.addCommands(
                new InstantCommand(intake::startIntake),
                new WaitCommand(500),
                new InstantCommand(intake::stopIntake),
                new WaitCommand(500));
        }
        return group;
    }

    @Override
    public void initialize() {
        prompter.prompt("alliance", new OptionPrompt<>("SELECT ALLIANCE", Alliance.RED, Alliance.BLUE))
                .prompt("parking", new OptionPrompt<>("SELECT PARKING LOCATION", Parking.CORNER_SIDE, Parking.FLOWER_SIDE))
                .onComplete(this::afterPromptsInitialize)
                .showSummary();


        LynxUtil.setBulkCachingMode(hardwareMap, LynxModule.BulkCachingMode.MANUAL);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        follower = Constants.create(hardwareMap);

        intake = new Intake(hardwareMap);
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

