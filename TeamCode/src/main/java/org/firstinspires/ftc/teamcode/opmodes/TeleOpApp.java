package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.api.Paths;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;
import com.skeletonarmy.marrow.BulkCaching;
import com.skeletonarmy.marrow.LynxUtil;
import com.pedropathing.paths.Path;

import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Intake;

@TeleOp(name="TeleOp")
@BulkCaching(mode = LynxModule.BulkCachingMode.MANUAL)
public class TeleOpApp extends CommandOpMode {
    private Follower follower;
    private Intake intake;
    private GamepadEx gamepad;
    private double halfRobotWidth = 17.785 / 2; //intake - new robot is 405 and 412 (yet to be checked which one is which)
    private double halfRobotLength = 17.323 / 2; //side
    private final Pose startPose = new Pose(halfRobotWidth, halfRobotLength, Math.toRadians(0));
    private boolean previousButtonB = false;
    private long lastPressButtonB = 0;
    private boolean isHoldingIntake = false;
    private boolean isToggledOn = false;

    private final int minThreshold = 150;
    private boolean currentButtonB;

    private final Pose flowerRedGarden = new Pose(15, 46, Math.toRadians(180));
    private final Pose flowerRedParking = new Pose(43, 124, Math.toRadians(90));
    private final Pose flowerBlueGarden = new Pose(123, 94, Math.toRadians(0));
    private final Pose flowerBlueParking = new Pose(93, 14, Math.toRadians(-90));

    private Pose calculateClosestFlower(){
        double robotX = follower.pose().x();
        double robotY = follower.pose().y();
        double distanceRedGarden = Math.hypot(flowerRedGarden.x() - robotX, flowerRedGarden.y() - robotY);
        double distanceRedParking = Math.hypot(flowerRedParking.x() - robotX, flowerRedParking.y() - robotY);
        double distanceBlueGarden = Math.hypot(flowerBlueGarden.x() - robotX, flowerBlueGarden.y() - robotY);
        double distanceBlueParking = Math.hypot(flowerBlueParking.x() - robotX, flowerBlueParking.y() - robotY);

        if (48 <= robotX && robotX <= 96 && 40 <= robotY && 104 >= robotY) {
            if (distanceBlueParking <  distanceRedParking) {
                return flowerBlueParking;
            }
            return flowerRedParking;
        }
        //checks redGarden
        if (distanceRedGarden < distanceRedParking &&
                distanceRedGarden < distanceBlueGarden &&
                distanceRedGarden < distanceBlueParking) {
            return flowerRedGarden;
        }
        //checks redParking
        else if (distanceRedParking < distanceBlueGarden &&
                distanceRedParking < distanceBlueParking) {
            return flowerRedParking;
        }
        //checks blueGarden
        else if (distanceBlueGarden < distanceBlueParking) {
            return flowerBlueGarden;
        }
        //checks blueParking
        return flowerBlueParking;
    }

    private void goToPose(Pose target) {
        Pose currPose = follower.pose();

        Path path = Paths.line(currPose, target).constant(target);

        follower.follow(path);
    }

    private void resetPosition() {
        double currentX = follower.pose().x();
        double currentY = follower.pose().y();
        double currentHeading = Math.toDegrees(follower.pose().heading());
        double roundedHeading = Math.round(currentHeading / 90) * 90;


        if (roundedHeading == 0 || roundedHeading == 180) {
            halfRobotWidth = halfRobotWidth;
            halfRobotLength = halfRobotLength;
        }
        else {
            double temp = halfRobotWidth;
            halfRobotWidth = halfRobotLength;
            halfRobotLength = temp;
        }

        boolean wallLeft = false;
        boolean wallClose = false;
        boolean wallFar = false;
        boolean wallRight = false;

        // WALLS:
        if(currentX < 5 + halfRobotWidth){
            wallLeft = true;
        }
        if(currentY < 5 + halfRobotLength){
            //follower.setPose(new Pose(currentX,0 + halfRobotLength,Math.toRadians(roundedHeading)));
            wallClose = true;

        }
        if(currentX > 139 - halfRobotWidth) {
            wallRight = true;

        }
        if (currentY > 139 - halfRobotLength) {
            //follower.setPose(new Pose(currentX, 144 - halfRobotLength, Math.toRadians(roundedHeading)));
            wallFar = true;

        }

        if (wallLeft) {
            if (wallClose) {
                follower.setPose(new Pose(0 + halfRobotWidth,0 + halfRobotLength,Math.toRadians(roundedHeading)));
                return;
            }

            if (wallFar) {
                follower.setPose(new Pose(0 + halfRobotWidth,144 - halfRobotLength,Math.toRadians(roundedHeading)));
            }
            else {
                follower.setPose(new Pose(0 + halfRobotWidth, currentY, Math.toRadians(roundedHeading)));
            }

            return;
        }

        if (wallRight) {
            if (wallClose) {
                follower.setPose(new Pose(144 - halfRobotWidth,0 + halfRobotLength,Math.toRadians(roundedHeading)));
                return;
            }

            if (wallFar) {
                follower.setPose(new Pose(144 - halfRobotWidth,144 - halfRobotLength,Math.toRadians(roundedHeading)));
            }
            else {
                follower.setPose(new Pose(144 - halfRobotWidth, currentY, Math.toRadians(roundedHeading)));
            }

            return;
        }
        if (wallFar){
            follower.setPose(new Pose(currentX,144 - halfRobotLength,Math.toRadians(roundedHeading)));
            return;
        }
        if (wallClose){
            follower.setPose(new Pose(currentX,0+halfRobotLength,Math.toRadians(roundedHeading)));
            return;
        }

    }

    private void goToReset() {
        double currentX = follower.pose().x();
        double currentY = follower.pose().y();
        int currentHeading = (int) Math.toDegrees(follower.pose().heading());
        double roundedHeading = Math.round(currentHeading / 90.0) * 90;


        if (roundedHeading == 0 || roundedHeading == 180) {
            halfRobotWidth = halfRobotWidth;
            halfRobotLength = halfRobotLength;
        }
        else {
            halfRobotWidth = 17.323 / 2;
            halfRobotLength = 17.785 / 2;
        }

        Pose targetPose;

        if (currentX <= 72 && currentY <= 72 ) {
            targetPose = new Pose(0 + halfRobotWidth,0 + halfRobotLength,Math.toRadians(roundedHeading));
        }
        else if (currentX > 72 && currentY < 72) {
            targetPose = new Pose(144 - halfRobotWidth,0 + halfRobotLength,Math.toRadians(roundedHeading));
        }
        else if (currentX < 72 && currentY > 72) {
            targetPose = new Pose(0 + halfRobotWidth,144 - halfRobotLength,Math.toRadians(roundedHeading));
        }
        else {
            targetPose = new Pose(144 - halfRobotWidth,144 - halfRobotLength,Math.toRadians(roundedHeading));
        }

        goToPose(targetPose);
    }

    private void driveManual() {
        follower.manual(
                -gamepad1.left_stick_y,
                -gamepad1.left_stick_x,
                -gamepad1.right_stick_x
        );
    }

    @Override
    public void initialize() {
        LynxUtil.clearBulkCache(hardwareMap);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        follower = Constants.create(hardwareMap);
        follower.setPose(startPose);

        intake = new Intake(hardwareMap);

        gamepad = new GamepadEx(gamepad1);

        gamepad.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whenHeld(new InstantCommand(intake::releaseIntake));
        gamepad.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whenReleased(new InstantCommand(intake::stopIntake));

        gamepad.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).whenPressed(new InstantCommand(() -> goToPose(calculateClosestFlower())));

        gamepad.getGamepadButton(GamepadKeys.Button.TRIANGLE).whenPressed(new InstantCommand(() -> resetPosition()));
        gamepad.getGamepadButton(GamepadKeys.Button.SQUARE).whenPressed(new InstantCommand(() -> goToReset()));

        gamepad.getGamepadButton(GamepadKeys.Button.LEFT_STICK_BUTTON).whenPressed(new InstantCommand(() -> driveManual()));
    }

    @Override
    public void run() {
        super.run();
        LynxUtil.clearBulkCache(hardwareMap);

        if (!follower.following()) {
            driveManual();
        }

        follower.update();

        currentButtonB = gamepad1.b;

        // 1. RISING EDGE: Button was just pressed down this loop cycle
        if (currentButtonB && !previousButtonB) {
            lastPressButtonB = System.currentTimeMillis();
            isHoldingIntake = false;
        }

        // 2. WHILE HELD: Button is currently being held down
        if (currentButtonB) {
            long passedTime = System.currentTimeMillis() - lastPressButtonB;

            // If held past your minThreshold, activate hold mode and start intake
            if (passedTime > minThreshold && !isHoldingIntake) {
                isHoldingIntake = true;
                schedule(new InstantCommand(intake::startIntake));
                //schedule(new InstantCommand(intake::startTransfer));

            }
        }
        // 3. FALLING EDGE: Button was just released this loop cycle
        else if (previousButtonB) {
            long passedTime = System.currentTimeMillis() - lastPressButtonB;

            // Short tap (under minThreshold): flip the toggle state
            if (passedTime <= minThreshold) {
                isToggledOn = !isToggledOn;
                if (isToggledOn) {
                    schedule(new InstantCommand(intake::startIntake));
                    //schedule(new InstantCommand(intake::startTransfer));
                } else {
                    schedule(new InstantCommand(intake::stopIntake));
                    //schedule(new InstantCommand(intake::stopTransfer));
                }
            }
            // Long press release: stop intake (unless the toggle is keeping it running)
            else if (isHoldingIntake) {
                isHoldingIntake = false;
                if (!isToggledOn) {
                    schedule(new InstantCommand(intake::stopIntake));
                    //schedule(new InstantCommand(intake::stopTransfer));
                }
            }
        }

        // Update the previous button state for the next loop iteration
        previousButtonB = currentButtonB;
        Pose closestFlower = calculateClosestFlower();
        String flowerName;
        boolean inMiddle = false;

        double robotX = follower.pose().x();
        double robotY = follower.pose().y();

        if (48 <= robotX && robotX <= 96 && 36 <= robotY && 108 >= robotY) {
            inMiddle = true;
        }

        if (closestFlower == flowerRedGarden) {
            flowerName = "RED GARDEN";
        }
        else if (closestFlower == flowerRedParking) {
            flowerName = "RED PARKING";
        }
        else if (closestFlower == flowerBlueGarden) {
            flowerName = "BLUE GARDEN";
        }
        else flowerName = "BLUE PARKING";

        telemetry.addData("location", follower.pose());
        telemetry.addData("flower", flowerName);
        telemetry.addData("middle", inMiddle);
        telemetry.addData("mode", follower.mode());
        telemetry.update();
    }
}
