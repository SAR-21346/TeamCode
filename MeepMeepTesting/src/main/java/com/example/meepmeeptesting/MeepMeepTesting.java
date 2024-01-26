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
                            return drive.trajectorySequenceBuilder(new Pose2d(-35, -59, Math.toRadians(0)))
                                    .lineTo(new Vector2d(-35, -53))
                                    .lineToSplineHeading(new Pose2d(-46.5, -35, Math.toRadians(90)))
                                    .waitSeconds(0.5)

                                    .back(6)

                                    .waitSeconds(0.5)
                                    .strafeRight(3)
                                    .splineToConstantHeading(new Vector2d(-34, -35), Math.toRadians(90))
                                    .splineToConstantHeading(new Vector2d(-34,-20), Math.toRadians(90))
                                    .splineToConstantHeading(new Vector2d(5, -13), Math.toRadians(0))
                                    .splineToSplineHeading(new Pose2d(52, -24, Math.toRadians(180)), Math.toRadians(0))

                                    .forward(6)
                                    .back(6)
                                    .resetAccelConstraint()
                                    .waitSeconds(0.5)

                                    .waitSeconds(0.5)
                                    .waitSeconds(1)
                                    .lineTo(new Vector2d(36, -45))
                                    .waitSeconds(0.4)
                                    .splineToConstantHeading(new Vector2d(60, -60), Math.toRadians(0))
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