package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-35, 61, Math.toRadians(270)))
                                .splineToConstantHeading(new Vector2d(-56.0, 35.0), Math.toRadians(180))
                                .lineToConstantHeading(new Vector2d(-46.0,38.0))
                                .splineTo(new Vector2d(-56.0,35.0), Math.toRadians(180))
                                .turn(Math.toRadians(180.0))
                                .splineToConstantHeading(new Vector2d(49.0,35.0), Math.toRadians(0.0))
                                .splineTo(new Vector2d(-24.0, 35.0), Math.toRadians(270))
                                .splineTo(new Vector2d(24,35), Math.toRadians(0))
                                .build()
                );
        
        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}