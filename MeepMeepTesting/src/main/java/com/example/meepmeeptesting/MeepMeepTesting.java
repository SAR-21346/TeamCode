package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

import java.util.Vector;

import jdk.incubator.vector.VectorOperators;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(30, 30, 5.02, 3.17, 18)
                .followTrajectorySequence(drive -> {
                            return drive.trajectorySequenceBuilder(new Pose2d(10.5, 59, Math.toRadians(0)))
                                    .lineToSplineHeading(new Pose2d(15, 45, Math.toRadians(180)))
                                    .splineToConstantHeading(new Vector2d(5, 29), Math.toRadians(180))
                                    .waitSeconds(.5)
                                    .addTemporalMarker(() -> {/**/})
                                    .addTemporalMarker(() -> {/**/})
                                    .back(4)
                                    .addTemporalMarker(() -> {/**/})
                                    .waitSeconds(0.5)
                                    .UNSTABLE_addDisplacementMarkerOffset(8, () -> {/**/})
                                    .lineToConstantHeading(new Vector2d(52, 27))
                                    //.setAccelConstraint(bot.odometry.SHAKE_ACCEL_CONSTRAINT)
                                    .forward(5)
                                    .back(5)
                                    .resetAccelConstraint()
                                    .waitSeconds(.5)
                                    .addTemporalMarker(() -> {/**/})
                                    .waitSeconds(.8)
                                    .addTemporalMarker(() -> {/**/})
                                    .addTemporalMarker(() -> {/**/})
                                    .waitSeconds(1)
                                    .lineTo(new Vector2d(45, 15))
                                    .addTemporalMarker(() -> {/**/})
                                    .waitSeconds(.4)
                                    .addTemporalMarker(() -> {/**/})
                                    .lineTo(new Vector2d(60, 7))
                                    .build();
                        }
                );


        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}