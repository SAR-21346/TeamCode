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
                            return drive.trajectorySequenceBuilder(new Pose2d(12, -59, Math.toRadians(180)))
                                    .lineTo(new Vector2d(12, -53))
//                                    .addTemporalMarker(() -> bot.target = 0)
//                                    .setVelConstraint(bot.odometry.SLOW_VEL_CONSTRAINT)
                                    .splineToConstantHeading(new Vector2d(5, -30), Math.toRadians(180))
                                    .waitSeconds(0.5)
//                                    .addTemporalMarker(() -> bot.openClaw())
                                    .back(6)
//                                    .addTemporalMarker(() -> bot.closeClaw())
                                    .waitSeconds(0.5)
                                    .resetVelConstraint()
//                .setAccelConstraint(bot.odometry.SHAKE_ACCEL_CONSTRAINT)
//                .forward(5)
//                .back(5)
//                .resetAccelConstraint()
//                                    .UNSTABLE_addDisplacementMarkerOffset(6, () -> bot.target = 138)
                                    .lineToConstantHeading(new Vector2d(52, -24))
                                    .waitSeconds(1)
//                                    .addTemporalMarker(() -> bot.openClaw())
                                    .waitSeconds(1)
//                                    .addTemporalMarker(() -> {
//                                        bot.target = 80;
//                                    })
//                                    .addTemporalMarker(() -> bot.closeClaw())
                                    .waitSeconds(1)
//                                    .setVelConstraint(bot.odometry.SLOW_VEL_CONSTRAINT)
//                                    .UNSTABLE_addTemporalMarkerOffset(0.8, () -> bot.target = 40)
//                                    .UNSTABLE_addTemporalMarkerOffset(1.2, () -> bot.target = 0)
                                    .lineTo(new Vector2d(34, -18))
                                    .splineToConstantHeading(new Vector2d(60, -5), Math.toRadians(0))
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