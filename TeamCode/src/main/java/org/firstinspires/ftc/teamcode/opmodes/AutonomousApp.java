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
import org.firstinspires.ftc.teamcode.enums.StartPosition;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Intake;

@Autonomous(name="Autonomous", preselectTeleOp="TeleOp")
public class AutonomousApp extends CommandOpMode {
    private final Prompter prompter = new Prompter(this);
    private Intake intake;
    private Alliance alliance;
    private Parking parking;
    private StartPosition startPosition;
    private Follower follower;
    private Path scorePathClose;
    private Path collectPath;
    private Path scorePathFar;
    private Path parkPath;

    private final Pose shootCloseStartPose = new Pose(63.3, 8.8925, Math.toRadians(180));
    private final Pose shootFarStartPose = new Pose(63.1075, 135.1075, Math.toRadians(180));
    private final Pose gardenStartPose = new Pose(8.8925, 32.6615, Math.toRadians(-90));

    private final Pose scorePoseClose = new Pose(63.3, 15, Math.toRadians(180));
    private final Pose collectPose = new Pose(13.5, 11.5, Math.toRadians(-95));
    private final Pose collectControlPoint = new Pose(30, 52, 0);
    private final Pose scorePoseFar = new Pose(49, 120, Math.toRadians(45));
    private final Pose scoreCloseControlPoint = new Pose(35, 53);
    private final Pose scoreFarControlPoint = new Pose(22, 120);
    private final Pose parkPoseCornerSide = new Pose(17 , 121, 0);
    private final Pose parkPoseFlowerSide = new Pose(16.5, 95, Math.toRadians(270));

    private final Pose hiveUpScorePose = new Pose(63.1075, 125, Math.toRadians(180));

    private final Pose gardenCollectPose = new Pose(8.8925, 11.1615, Math.toRadians(-90));

    public void afterPromptsInitialize() {
        parking = prompter.getOrDefault("parking", Parking.CORNER_SIDE);
        alliance = prompter.getOrDefault("alliance", Alliance.RED);
        startPosition = prompter.getOrDefault("startPose", StartPosition.SHOOT_CLOSE);

        scorePathClose = getScorePathClose(alliance);
        collectPath = getCollectPath(alliance);
        scorePathFar = getScorePathFar(alliance);
        parkPath = getParkPath(alliance, parking);

        if (startPosition == StartPosition.SHOOT_CLOSE){
            follower.setPose(changeAlliance(shootCloseStartPose, alliance));
            afterPromptsShootClose();
        }
        else if (startPosition == StartPosition.SHOOT_FAR){
            follower.setPose(changeAlliance(shootFarStartPose, alliance));
            afterPromptsShootFar();
        }
        else if (startPosition == StartPosition.GARDEN){
            follower.setPose(changeAlliance(gardenStartPose, alliance));
            afterPromptsGarden();
        }
    }
    public void afterPromptsShootClose() {
        schedule(
                new SequentialCommandGroup(
                        new FollowPathCommand(follower, scorePathClose),
                        new InstantCommand(), // Shoot
                        new WaitCommand(2000),

                        new FollowPathCommand(follower, collectPath),
                        //new InstantCommand(intake::startTransfer),
                        //pulseIntake(4),
                        //new InstantCommand(intake::stopTransfer),

                        new FollowPathCommand(follower, scorePathFar),
                        new InstantCommand(), // shoot
                        new WaitCommand(1500),

                        new FollowPathCommand(follower, parkPath),
                        new WaitCommand(1000)
                )
        );
    }
    public void afterPromptsShootFar() {
        schedule(
                new SequentialCommandGroup(

                        new FollowPathCommand(follower, getScoreFarToShootFarPath(alliance)),
                        new InstantCommand(), // shoot
                        new WaitCommand(1500),

                        new FollowPathCommand(follower, parkPath),
                        new WaitCommand(1000)
                )
        );
    }
    public void afterPromptsGarden() {
        schedule(
                new SequentialCommandGroup(
                        new FollowPathCommand(follower, getGardenCollectPath(alliance)),
                        new InstantCommand(intake::startTransfer),
                        pulseIntake(4),
                        new InstantCommand(intake::stopTransfer),

                        new FollowPathCommand(follower, getGardenToScoreFarPath(alliance)),
                        new InstantCommand(), // shoot
                        new WaitCommand(1500),

                        new FollowPathCommand(follower, parkPath),
                        new WaitCommand(1000)
                )
        );
    }
    public static Pose changeAlliance(Pose pose, Alliance alliance) {
        if (alliance == Alliance.BLUE) {
            return mirrorPose(pose);
        }
        return pose;
    }
        public static Pose mirrorPose(Pose pose) {
        double newX = 144 - pose.x();
        double newY = 144 -pose.y();
        double newHeading = pose.heading() - Math.toRadians(180);

        return new Pose(newX, newY, newHeading);
    }

    private Path getScorePathClose(Alliance alliance) {
        return line(
                changeAlliance(shootCloseStartPose, alliance),
                changeAlliance(scorePoseClose, alliance)
        ).constant(changeAlliance(shootCloseStartPose, alliance));
    }
    private Path getScoreFarToShootFarPath(Alliance alliance) {
        return line(
                changeAlliance(shootFarStartPose, alliance),
                changeAlliance(hiveUpScorePose, alliance)
        ).constant(changeAlliance(shootFarStartPose, alliance));
    }
    private Path getShootFarToParkPath(Alliance alliance, Parking parking) {
        if (parking == Parking.CORNER_SIDE) {
            return line(
                    changeAlliance(hiveUpScorePose, alliance),
                    changeAlliance(parkPoseCornerSide, alliance)
            ).linear(changeAlliance(hiveUpScorePose, alliance) , changeAlliance(parkPoseCornerSide, alliance));
        }
        else if (parking == Parking.FLOWER_SIDE){
            return line(
                    changeAlliance(hiveUpScorePose, alliance),
                    changeAlliance(parkPoseFlowerSide, alliance)
            ).linear(changeAlliance(hiveUpScorePose, alliance), changeAlliance(parkPoseFlowerSide, alliance));
        }
        return null;
    }
    private Path getGardenToScoreFarPath(Alliance alliance) {
        return curve(
                changeAlliance(gardenCollectPose, alliance),
                changeAlliance(scoreCloseControlPoint, alliance),
                changeAlliance(scoreFarControlPoint, alliance),
                changeAlliance(scorePoseFar, alliance)
        ).constant(changeAlliance(gardenCollectPose, alliance));
    }
    private Path getGardenCollectPath(Alliance alliance) {
        return line(
                changeAlliance(gardenStartPose, alliance),
                changeAlliance(gardenCollectPose, alliance)
        ).constant(changeAlliance(gardenStartPose, alliance));
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
                changeAlliance(scoreCloseControlPoint, alliance),
                changeAlliance(scoreFarControlPoint, alliance),
                changeAlliance(scorePoseFar, alliance)
        ).constant(changeAlliance(collectPose, alliance));
    }

    private Path getParkPath(Alliance alliance, Parking parking) {
        if (parking == Parking.CORNER_SIDE) {
            return line(
                    changeAlliance(scorePoseFar, alliance),
                    changeAlliance(parkPoseCornerSide, alliance)
            ).linear(changeAlliance(collectPose, alliance) , changeAlliance(parkPoseCornerSide, alliance));
        }
        else if (parking == Parking.FLOWER_SIDE){
            return line(
                    changeAlliance(scorePoseFar, alliance),
                    changeAlliance(parkPoseFlowerSide, alliance)
            ).linear(changeAlliance(collectPose, alliance), changeAlliance(parkPoseFlowerSide, alliance));
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
                .prompt("startPose", new OptionPrompt<>("SELECT START POSE", StartPosition.SHOOT_CLOSE, StartPosition.SHOOT_FAR, StartPosition.GARDEN))
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

