package org.firstinspires.ftc.teamcode.consts;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;

public class HivePositions {
    private final static double FIELD_LENGTH = 144;
    private final static double HIVE_X = 57.5;

    public final static Pose AUDIENCE_HIVE = new Pose(HIVE_X,55);
    public final static Pose REAR_HIVE = new Pose(HIVE_X, FIELD_LENGTH - AUDIENCE_HIVE.y());

    //todo: fix hive positions ^^^

    public static Pose getHivePose(Follower follower){
        if (follower.pose().y() < 72)
            return AUDIENCE_HIVE;
        return REAR_HIVE;
    }
}
