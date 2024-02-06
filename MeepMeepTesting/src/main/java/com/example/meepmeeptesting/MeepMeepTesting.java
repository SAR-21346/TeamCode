package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.ColorScheme;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeBlueDark;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedDark;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;
public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);
        Pose2d redFrontStart = new Pose2d(12, -59, Math.toRadians(180));
        Pose2d blueFrontStart = new Pose2d(12, 59, Math.toRadians(0));
        Pose2d redBackStart = new Pose2d(-35, -59, Math.toRadians(180));
        Pose2d blueBackStart = new Pose2d(-35, 59, Math.toRadians(0));
        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(20, 50, 5.02, 3.17, 18)
                .setColorScheme(new ColorSchemeBlueDark())
                .followTrajectorySequence(drive -> {
                            return drive.trajectorySequenceBuilder(redBackStart)
                                    .lineTo(new Vector2d(-40, -53))
                                    .addTemporalMarker(() -> {/**/})
                                    .addTemporalMarker(() -> {/**/})
                                    .lineTo(new Vector2d(-40, -55))
                                    .waitSeconds(1)
                                    .lineToSplineHeading(new Pose2d(-39, -37, Math.toRadians(0)))
                                    .splineToConstantHeading(new Vector2d(-28, -34), Math.toRadians(180))
                                    .addTemporalMarker(() -> {/**/})
                                    .waitSeconds(0.5)
                                    .back(5)
                                    .addTemporalMarker(() -> {/**/})
                                    .waitSeconds(0.5)
                                    .back(8)
//                .setAccelConstraint(bot.odometry.SHAKE_ACCEL_CONSTRAINT)
//                .forward(4)
//                .back(4)
//                .resetAccelConstraint()
                                    .lineTo(new Vector2d(-40, -59)).build();
                        }
                );


        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}