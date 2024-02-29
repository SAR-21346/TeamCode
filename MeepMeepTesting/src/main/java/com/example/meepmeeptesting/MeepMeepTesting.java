package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeBlueDark;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;
public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);
        Pose2d redFrontStart = new Pose2d(8.5, -62, Math.toRadians(180));
        Pose2d blueFrontStart = new Pose2d(12, 59, Math.toRadians(0));
        Pose2d redBackStart = new Pose2d(-35, -59, Math.toRadians(180));
        Pose2d blueBackStart = new Pose2d(-35, 59, Math.toRadians(0));
        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(40, 50, 5.02, 3.17, 20.5)
                .setColorScheme(new ColorSchemeBlueDark())
                .followTrajectorySequence(drive -> {
                            return drive.trajectorySequenceBuilder(redFrontStart)
                                    .addTemporalMarker(() -> {/**/})
                                    .addTemporalMarker(() -> {/**/})
                                    .lineTo(new Vector2d(15, -53))
                                    .waitSeconds(0.25)
                                    .lineTo(new Vector2d(25, -48))
//                                    .splineToConstantHeading(new Vector2d(21, -48), Math.toRadians(90))
                                    .splineToConstantHeading(new Vector2d(29.5, -33), Math.toRadians(90))
                                    .addTemporalMarker(() -> {/**/})
                                    .waitSeconds(0.5)
                                    .back(4)
                                    .addTemporalMarker(() -> {/**/})
                                    .waitSeconds(0.5)
                                    .back(8)
                                    //.setConstraints(bot.odometry.SHAKE_VEL_CONSTRAINT, bot.odometry.SHAKE_ACCEL_CONSTRAINT)
                                    .forward(4)
                                    //.back(6)
                                    .waitSeconds(0.5)
                                    .resetConstraints()
                                    .UNSTABLE_addDisplacementMarkerOffset(4, () -> {/**/})
                                    .lineTo(new Vector2d(44, -36))
                                    .splineToConstantHeading(new Vector2d(52, -37), Math.toRadians(0))
                                    .addTemporalMarker(() -> {/**/})
                                    .waitSeconds(0.3)
                                    .addTemporalMarker(() -> {/**/})
                                    .addTemporalMarker(() -> {/**/})
                                    .waitSeconds(0.4)
                                    .UNSTABLE_addTemporalMarkerOffset(0.8, () -> {/**/})
                                    .UNSTABLE_addTemporalMarkerOffset(1.2, () -> {/**/})
                                    .lineTo(new Vector2d(35, -12))
                                    .lineTo(new Vector2d(-59, -11))
                                    .addTemporalMarker(() -> {/**/})
                                    .addTemporalMarker(() -> {/**/})
                                    .waitSeconds(0.5)
                                    .addTemporalMarker(() -> {/**/})
                                    .addTemporalMarker(() -> {/**/})
                                    .lineTo(new Vector2d(35, -12))
                                    .addTemporalMarker(() -> {/**/})
                                    .addTemporalMarker(() -> {/**/})
                                    .splineToConstantHeading(new Vector2d(52, -29), Math.toRadians(0))
                                    .addTemporalMarker(() -> {/**/})
                                    .waitSeconds(0.3)
                                    .addTemporalMarker(() -> {/**/})
                                    .addTemporalMarker(() -> {/**/})
                                    .waitSeconds(0.4)
                                    .UNSTABLE_addTemporalMarkerOffset(0.8, () -> {/**/})
                                    .UNSTABLE_addTemporalMarkerOffset(1.2, () -> {/**/})
                                    .forward(5)
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