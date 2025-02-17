package org.firstinspires.ftc.teamcode.autonomous;

import com.pedropathing.localization.Pose;


public class FieldConstants {
    // Spike Mark Locations
    public static final Pose neutralLeftSpike = new Pose(45.5, 144-2.5);
    public static final Pose neutralCenterSpike = new Pose(45.5, 144-12.5);
    public static final Pose neutralRightSpike = new Pose(45.5, 144-22.5);

    public static final Pose blueRightSpike = new Pose(45.5, 2);
    public static final Pose blueCenterSpike = new Pose(45.5, 2+10.5);
    public static final Pose blueLeftSpike = new Pose(45.5, 2+10.5+10.5);

    // Basket Park Locations
    public static final Pose basket = new Pose(16, 128, Math.toRadians(-45));

    // Start Locations
    public static final Pose basketStart = new Pose(8, 108.5, Math.toRadians(0));

    // Park Locations
    public static final Pose ascentParkingBlue = new Pose(60, 96, Math.toRadians(90));
}
