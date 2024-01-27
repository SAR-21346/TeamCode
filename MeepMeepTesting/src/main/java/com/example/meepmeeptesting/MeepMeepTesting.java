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
        ColorScheme ColorSchemeRedDark = new ColorSchemeRedDark();
        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(50, 50, 5.02, 3.17, 18)
                .setColorScheme(ColorSchemeRedDark)
                .followTrajectorySequence(drive -> {
                            return drive.trajectorySequenceBuilder(new Pose2d(10.5, 60, Math.toRadians(0)))
                                    .lineTo(new Vector2d(12, 53))
                                    .splineToSplineHeading(new Pose2d(23, 38, Math.toRadians(270)), Math.toRadians(135))
                                    .waitSeconds(0.5)
                                    .addTemporalMarker(() -> {/**/})
                                    .back(3)
                                    .waitSeconds(0.5)
                                    .addTemporalMarker(() -> {/**/})
                                    .waitSeconds(0.5)
                                    .back(9)
                                    .UNSTABLE_addDisplacementMarkerOffset(6, () -> {/**/})
                                    //.setAccelConstraint(bot.odometry.SHAKE_ACCEL_CONSTRAINT)
                                    .forward(4)
                                    .back(4)
                                    .resetAccelConstraint()
                                    .lineTo(new Vector2d(25, 43))
                                    .splineToSplineHeading(new Pose2d(53, 35, Math.toRadians(180)), Math.toRadians(0))
                                    .waitSeconds(0.5)
                                    .addTemporalMarker(() -> {/**/})
                                    .waitSeconds(0.5)
                                    .addTemporalMarker(() -> {/**/})
                                    .addTemporalMarker(() -> {/**/})
                                    .waitSeconds(0.5)
                                    .lineTo(new Vector2d(45, 15))
                                    .addTemporalMarker(() -> {/**/})
                                    .waitSeconds(0.3)
                                    .addTemporalMarker(() -> {/**/})
                                    .lineTo(new Vector2d(60, 10))
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