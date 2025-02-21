package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.localization.Pose;

@Config
public class FieldConstants {
    // Spike Mark Locations
    public static Pose neutralLeftSpike = new Pose(49, 135, Math.toRadians(90));
    public static Pose neutralCenterSpike = new Pose(20, 134, Math.toRadians(0));
    public static Pose neutralRightSpike = new Pose(20, 124, Math.toRadians(0));

    public static Pose blueRightSpike = new Pose(45.5, 2);
    public static Pose blueCenterSpike = new Pose(45.5, 2+10.5);
    public static Pose blueLeftSpike = new Pose(45.5, 2+10.5+10.5);

    // Basket Park Locations
    public static Pose basket = new Pose(16.5, 126.5, Math.toRadians(315));

    // Start Locations
    public static Pose basketStart = new Pose(8, 108.5, Math.toRadians(0));

    // Park Locations
    public static Pose ascentParkingBlue = new Pose(60, 96, Math.toRadians(90));
}
