package org.firstinspires.ftc.teamcode.config;

import com.skeletonarmy.marrow.zones.Point;
import com.skeletonarmy.marrow.zones.PolygonZone;

public class zones {

    Point ZonePoint1 = new Point(0,0);
    Point ZonePoint2 =new Point(2.2213, 25.6179);
    Point ZonePoint3 =new Point(56.528, 50.2388);
    Point ZonePoint4 =new Point(140.0254, 13.4743);
    Point ZonePoint5 =new Point(144,0);
    public final int FIELD_LENGTH = 144;
    PolygonZone shootingZoneRedClose = new PolygonZone(
            ZonePoint1,
            ZonePoint2,
            ZonePoint3,
            ZonePoint4,
            ZonePoint5
    );

    PolygonZone shootingZoneRedFar = new PolygonZone(
            new Point(ZonePoint1.getX(),FIELD_LENGTH - ZonePoint1.getY()),
            new Point(ZonePoint2.getX(),FIELD_LENGTH - ZonePoint2.getY()),
            new Point(ZonePoint3.getX(),FIELD_LENGTH - ZonePoint3.getY()),
            new Point(ZonePoint4.getX(),FIELD_LENGTH - ZonePoint4.getY()),
            new Point(ZonePoint5.getX(),FIELD_LENGTH - ZonePoint5.getY())
    );

    PolygonZone shootingZoneBlueClose = new PolygonZone(
            new Point(FIELD_LENGTH - ZonePoint1.getX(), ZonePoint1.getY()),
            new Point(FIELD_LENGTH - ZonePoint2.getX(), ZonePoint2.getY()),
            new Point(FIELD_LENGTH - ZonePoint3.getX(), ZonePoint3.getY()),
            new Point(FIELD_LENGTH - ZonePoint4.getX(), ZonePoint4.getY()),
            new Point(FIELD_LENGTH - ZonePoint5.getX(), ZonePoint5.getY())
    );

    PolygonZone shootingZoneBlueFar = new PolygonZone(
            new Point(FIELD_LENGTH - ZonePoint1.getX(),FIELD_LENGTH - ZonePoint1.getY()),
            new Point(FIELD_LENGTH - ZonePoint2.getX(),FIELD_LENGTH - ZonePoint2.getY()),
            new Point(FIELD_LENGTH - ZonePoint3.getX(),FIELD_LENGTH - ZonePoint3.getY()),
            new Point(FIELD_LENGTH - ZonePoint4.getX(),FIELD_LENGTH - ZonePoint4.getY()),
            new Point(FIELD_LENGTH - ZonePoint5.getX(),FIELD_LENGTH - ZonePoint5.getY())
    );
}
