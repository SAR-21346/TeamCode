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
                                    .lineTo(new Vector2d(14, -44))
                                    .splineToSplineHeading(new Pose2d(4, -38.5, Math.toRadians(110)), Math.toRadians(180))
                                    .addDisplacementMarker(() -> {/**/})
                                    .waitSeconds(0.5)
                                    .back(2)
                                    .addDisplacementMarker(() -> {/**/})
                                    .waitSeconds(0.5)
                                    .lineTo(new Vector2d(14, -44))
                                    //.setConstraints(bot.odometry.SHAKE_VEL_CONSTRAINT, bot.odometry.SHAKE_ACCEL_CONSTRAINT)
                                    .forward(5)
                                    .back(5)
                                    .resetConstraints()
                                    .addDisplacementMarker(() -> {
//                                        bot.runLift(200);
//                                        bot.target = 150;
                                    })
                                    .splineToSplineHeading(new Pose2d(48, -28, Math.toRadians(180)), Math.toRadians(0))
                                    .addDisplacementMarker(() -> {/**/})
                                    .waitSeconds(0.5)
                                    .addDisplacementMarker(() -> {
//                                        bot.closeClaw();
//                                        bot.target = 0;
//                                        bot.runLift(0);
                                    })
                                    .lineTo(new Vector2d(45, -15))
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