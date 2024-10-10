package org.firstinspires.ftc.teamcode.autonomous;

import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;


public class FieldConstants {
    // Spike Mark Locations
    public static final Pose blueAllianceNeutralLeftSpike = new Pose(44.5, 144-2.5);
    public static final Pose blueAllianceNeutralCenterSpike = new Pose(44.5, 144-12.5);
    public static final Pose blueAllianceNeutralRightSpike = new Pose(44.5, 144-22.5);

    public static final Pose redAllianceRedRightSpike = new Pose(144-44.5, 144-2.5);
    public static final Pose redAllianceRedCenterSpike = new Pose(144-44.5, 144-12.5);
    public static final Pose redAllianceRedLeftSpike = new Pose(144-44.5, 144-22.5);

    public static final Pose blueAllianceBlueRightSpike = new Pose(44.5, 2);
    public static final Pose blueAllianceBlueCenterSpike = new Pose(44.5, 2+10.5);
    public static final Pose blueAllianceBlueLeftSpike = new Pose(44.5, 2+10.5+10.5);

    public static final Pose redAllianceNeutralRightSpike = new Pose(44.5+24*2+4.5, 2);
    public static final Pose redAllianceNeutralCenterSpike = new Pose(44.5+24*2+4.5, 2+10.5);
    public static final Pose redAllianceNeutralLeftSpike = new Pose(44.5+24*2+4.5, 2+10.5+10.5);

    // Basket Park Locations
    public static final Pose blueAllianceBasket = new Pose(15, 130, Math.toRadians(-45));
    public static final Pose redAllianceBasket = new Pose(130, 15, Math.toRadians(135));

    // Start Locations
    public static final Pose blueAllianceBasketStart = new Pose(10, 84, Math.toRadians(0));

    // Park Locations
    public static final Pose ascentParking = new Pose(60, 96, Math.toRadians(90));
}
