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
                            return drive.trajectorySequenceBuilder(new Pose2d(10.5, -59, Math.toRadians(180)))
                                    .lineTo(new Vector2d(10.5, -53))
                                    .lineTo(new Vector2d(14, -40))
                                    .splineToConstantHeading(new Vector2d(3, -32), Math.toRadians(180))
                                    .waitSeconds(0.5)
                                    .addTemporalMarker(() -> {/**/})
                                    .waitSeconds(0.5)
                                    .back(5)
                                    .addTemporalMarker(() -> {/**/})
                                    .lineTo(new Vector2d(14, -44))
                                    .addTemporalMarker(() -> {/**/})
                                    .lineToConstantHeading(new Vector2d(52, -42))
                                    .waitSeconds(1)
                                    .addTemporalMarker(() -> {/**/})
                                    .waitSeconds(1)
                                    .addTemporalMarker(() -> {/**/})
                                    .waitSeconds(2)
                                    .lineTo(new Vector2d(45, -15))
                                    .addTemporalMarker(() -> {/**/})
                                    .waitSeconds(0.8)
                                    .addTemporalMarker(() -> {/**/})
                                    .waitSeconds(0.8)
                                    .addTemporalMarker(() -> {/**/})
                                    .splineToConstantHeading(new Vector2d(60, -12.5), Math.toRadians(0))
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