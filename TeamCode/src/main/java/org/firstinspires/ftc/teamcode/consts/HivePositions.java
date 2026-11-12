package org.firstinspires.ftc.teamcode.consts;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;

public class HivePositions {
    private static final double FIELD_LENGTH = 144;
    private static final double HIVE_X = 57.5;
    private static final double HIVE_Y = 55;
    public static final Pose AUDIENCE_HIVE = new Pose(HIVE_X, HIVE_Y);
    public static final Pose REAR_HIVE = new Pose(HIVE_X, FIELD_LENGTH - HIVE_Y);

    public static Pose getHivePose(double robotY) {
        if (robotY < 72) return AUDIENCE_HIVE;
        return REAR_HIVE;
    }
}
