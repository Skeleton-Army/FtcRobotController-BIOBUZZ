package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.controllers.Controller;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Matrix;
import com.pedropathing.math.Vector2D;
import com.pedropathing.revhub.drivetrains.CoaxialPodConfig;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.pedropathing.revhub.localizers.PinpointConfig;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Constants {

    public static MecanumConfig drivetrainConfig = new MecanumConfig(c -> {
        c.frontLeftName.set("leftFront");
        c.frontRightName.set("rightFront");
        c.backLeftName.set("leftBack");
        c.backRightName.set("rightBack");
        c.frontLeftDirection.set(DcMotorSimple.Direction.REVERSE);
        c.frontRightDirection.set(DcMotorSimple.Direction.FORWARD);
        c.backLeftDirection.set(DcMotorSimple.Direction.REVERSE);
        c.backRightDirection.set(DcMotorSimple.Direction.FORWARD);
    });

    public static PinpointConfig localizerConfig = new PinpointConfig(c -> {
        c.name.set("pinpoint");
        c.podType.set(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        c.xPodOffset.set(-0.254528);
        c.yPodOffset.set(-6.889764);
        c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.REVERSED);
        c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
        c.globalDistanceUnit.set(DistanceUnit.INCH);
        c.offsetUnits.set(DistanceUnit.INCH);
    });

    public static ForesightConfig foresightConfig = new ForesightConfig(
            c -> {
                Controller primaryTranslationalForward = Controller.proportional(0.2707654531023506);
                Controller secondaryTranslationalForward = Controller.proportional(0.100040599935163);
                Controller primaryTranslationalLateral = Controller.proportional(0.29912328947056804);
                Controller secondaryTranslationalLateral = Controller.proportional(0.11051806273787619);

                c.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward));
                c.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral).put(2.5, primaryTranslationalLateral));

                c.coast.set(Controller.proportionalFeedforward(0.012926151967860304));
                c.brake.set(Controller.proportionalFeedforward(0.010987229172681258));

                c.headingFeedback.set(Controller.proportional(4.455182145837618));
                c.headingBrakeCoefficients.set(Vector2D.cartesian(0.050618831268814835, 0.00650969227888265));

                c.linearBrakeCoefficients.set(Matrix.diag(0.053354363924609474, 0.06491437188668317));
                c.quadraticBrakeCoefficients.set(Matrix.diag(0.002032750945781706, 0.0016051278992185855));

                c.maxAchievableForwardVelocity.set(75.31267925550661);
                c.maxAchievableStrafeVelocity.set(63.71499218955991);
                c.naturalForwardDeceleration.set(34.410346757401655);
                c.naturalStrafeDeceleration.set(60.22854070551005);
            }
    );

    public static Follower create(HardwareMap h) {
        localizerConfig.resetMode.set(PinpointLocalizer.ResetMode.RECALIBRATE_IMU);

        return new Follower(
                new PinpointLocalizer(h, localizerConfig),
                new Mecanum(h, drivetrainConfig),
                new Foresight(foresightConfig)
        );
    }

    /**
     * Creates a follower that doesn't reset the pinpoint pose at the start.
     * This is useful for TeleOp when the robot continues off where the autonomous ended.
     */
    public static Follower createWithoutReset(HardwareMap h) {
        localizerConfig.resetMode.set(PinpointLocalizer.ResetMode.NONE);

        return new Follower(
                new PinpointLocalizer(h, localizerConfig),
                new Mecanum(h, drivetrainConfig),
                new Foresight(foresightConfig)
        );
    }
}