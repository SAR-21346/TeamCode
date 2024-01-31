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
        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(50, 50, 5.02, 3.17, 18)
                .setColorScheme(new ColorSchemeBlueDark())
                .followTrajectorySequence(drive -> {
                            return drive.trajectorySequenceBuilder(new Pose2d(-35, 59, Math.toRadians(0)))
                                    .lineTo(new Vector2d(-40, 50))
                                    .splineToSplineHeading(new Pose2d(-41, 24, Math.toRadians(315)), Math.toRadians(315))
                                    .waitSeconds(0.5)
                                    .addTemporalMarker(() -> {/**/})
                                    .back(6)
                                    .addTemporalMarker(() -> {/**/})
                                    .waitSeconds(0.5)
                                    .back(4)
                                    //.setAccelConstraint(bot.odometry.SHAKE_ACCEL_CONSTRAINT)
                                    .forward(4)
                                    .back(4)
                                    .resetAccelConstraint()
                                    .splineToSplineHeading(new Pose2d(-40, 59, Math.toRadians(0)), Math.toRadians(0))
                                    .build();
                        }
                );


        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}