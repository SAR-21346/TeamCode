package org.firstinspires.ftc.teamcode.autonomous;

import com.pedropathing.pathgen.PathBuilder;
import com.pedropathing.pathgen.Point;
import com.pedropathing.pathgen.PathBuilder;


import com.pedropathing.localization.Pose;

public class FieldConstants {
    // Spike Mark Locations
    public static final Pose blueAllianceSpecStart = new Pose(8, 59.375, 0);
    public static final Point blueAlliancePreloadSpec = new Point(38, 72);

    public static final Point blueAllianceSpec1 = new Point(38, 70);
    public static final Point blueAllianceSpec1turn = new Point(38, 70);
    public static final Point blueAllianceSpec2 = new Point(38, 67);
    public static final Point blueAllianceSpec3 = new Point(38, 64);


    public static final Point blueAllianceSample1 = new Point(35, 35);
    public static final Point blueAllianceSample2 = new Point(35, 27);
    public static final Point blueAllianceSample3 = new Point(35, 18);
    public static final Point blueAllianceSpecPickup = new Point(35, 18);








    public static final Pose blueAllianceNeutralLeftSpike = new Pose(45.5, 144-2.5);
    public static final Pose blueAllianceNeutralCenterSpike = new Pose(45.5, 144-12.5);
    public static final Pose blueAllianceNeutralRightSpike = new Pose(45.5, 144-22.5);

    public static final Pose redAllianceRedRightSpike = new Pose(144-45.5, 144-2.5);
    public static final Pose redAllianceRedCenterSpike = new Pose(144-45.5, 144-12.5);
    public static final Pose redAllianceRedLeftSpike = new Pose(144-45.5, 144-22.5);

    public static final Pose blueAllianceBlueRightSpike = new Pose(45.5, 2);
    public static final Pose blueAllianceBlueCenterSpike = new Pose(45.5, 2+10.5);
    public static final Pose blueAllianceBlueLeftSpike = new Pose(45.5, 2+10.5+10.5);

    public static final Pose redAllianceNeutralRightSpike = new Pose(45.5+24*2+4.5, 2);
    public static final Pose redAllianceNeutralCenterSpike = new Pose(45.5+24*2+4.5, 2+10.5);
    public static final Pose redAllianceNeutralLeftSpike = new Pose(45.5+24*2+4.5, 2+10.5+10.5);

    // Scoring Locations
    public static final Pose blueAllianceBasket = new Pose(13, 120, Math.toRadians(-60));
    public static final Pose blueAllianceSpec = new Pose(38, 72, Math.toRadians(0));
    public static final Pose redAllianceBasket = new Pose(122, 15, Math.toRadians(135));
//    public static final Pose redAllianceBasket = new Pose(122, 15, Math.toRadians(135)); FIX THIS LINE


    // Start Locations
    public static final Pose blueAllianceBasketStart = new Pose(8, 106, Math.toRadians(0));
    public static final Pose redAllianceBasketStart = new Pose (136, 60, Math.toRadians(180));

    // Park Locations
    public static final Pose ascentParkingBlue = new Pose(63, 96, Math.toRadians(90));
    public static final Pose ascentParkingRed = new Pose(84, 48, Math.toRadians(270));
}
