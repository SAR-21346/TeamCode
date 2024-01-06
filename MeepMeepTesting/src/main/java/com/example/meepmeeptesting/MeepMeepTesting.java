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
                                    .lineTo(new Vector2d(12, -53))
                                    .splineToLinearHeading(new Pose2d(23, -41, Math.toRadians(90)), Math.toRadians(0))
                                    .addDisplacementMarker(() -> {/**/})
                                    .back(3)
                                    .addDisplacementMarker(() -> {/**/})
                                    .back(5)
                                    .lineToSplineHeading(new Pose2d(35, -34.5, Math.toRadians(90)))
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