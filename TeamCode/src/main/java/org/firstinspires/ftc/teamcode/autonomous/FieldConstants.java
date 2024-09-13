package org.firstinspires.ftc.teamcode.autonomous;

import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;

public class FieldConstants {
    public static final Pose blueAllianceNeutralRightSpike = new Pose(45.5, 121.25);
    public static final Pose blueAllianceNeutralCenterSpike = new Pose(45.5, 121.25+10.5);
    public static final Pose blueAllianceNeutralLeftSpike = new Pose(45.5, 121.25+10.5+10.5);

    public static final Pose redAllianceRedRightSpike = new Pose(45.5+24*2+4.5, 121.25);
    public static final Pose redAllianceRedCenterSpike = new Pose(45.5+24*2+4.5, 121.25+10.5);
    public static final Pose redAllianceRedLeftSpike = new Pose(45.5+24*2+4.5, 121.25+10.5+10.5);

    public static final Pose blueAllianceBlueRightSpike = new Pose(45.5, 2);
    public static final Pose blueAllianceBlueCenterSpike = new Pose(45.5, 2+10.5);
    public static final Pose blueAllianceBlueLeftSpike = new Pose(45.5, 2+10.5+10.5);

    public static final Pose redAllianceNeutralRightSpike = new Pose(45.5+24*2+4.5, 2);
    public static final Pose redAllianceNeutralCenterSpike = new Pose(45.5+24*2+4.5, 2+10.5);
    public static final Pose redAllianceNeutralLeftSpike = new Pose(45.5+24*2+4.5, 2+10.5+10.5);
}
