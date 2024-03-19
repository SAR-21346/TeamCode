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
        Pose2d redFrontStart = new Pose2d(9.5, -59, Math.toRadians(180));
        Pose2d blueFrontStart = new Pose2d(12, 59, Math.toRadians(0));
        Pose2d redBackStart = new Pose2d(-35, -59, Math.toRadians(180));
        Pose2d blueBackStart = new Pose2d(-35, 59, Math.toRadians(0));
        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(40, 80, 5.02, 3.17, 20.5)
                .setColorScheme(new ColorSchemeBlueDark())
                .followTrajectorySequence(drive -> {
                            return drive.trajectorySequenceBuilder(blueFrontStart)
                                    .addTemporalMarker(() -> {/**/}) // close claw
                                    .addTemporalMarker(() -> {/**/}) // target = 0
                                    .lineTo(new Vector2d(18, 53))
                                    .waitSeconds(0.25)
                                    //.setVelConstraint(bot.odometry.SLOW_VEL_CONSTRAINT)
                                    .lineTo(new Vector2d(14, 40))
                                    .splineToConstantHeading(new Vector2d(23, 35), Math.toRadians(0))
                                    .waitSeconds(0.5)
                                    .addTemporalMarker(() -> {/**/}) // open claw
                                    .back(4)
                                    .addTemporalMarker(() -> {/**/}) // close claw
                                    .waitSeconds(0.5)
                                    .UNSTABLE_addDisplacementMarkerOffset(15, () -> {{/**/}})
                                    .back(6)
                                    //.setConstraints(bot.odometry.SHAKE_VEL_CONSTRAINT, bot.odometry.SHAKE_ACCEL_CONSTRAINT)
                                    .forward(4)
                                    .waitSeconds(0.5)
                                    .resetConstraints()
                                    .lineTo(new Vector2d(24, 52))
                                    .splineToSplineHeading(new Pose2d(52, 37, Math.toRadians(180)), Math.toRadians(0))
                                    .addTemporalMarker(() -> {/**/}) // open claw
                                    .waitSeconds(0.5)
                                    .addTemporalMarker(() -> {{/**/};}) // target = 80
                                    .addTemporalMarker(() -> {/**/}) // close claw
                                    .UNSTABLE_addTemporalMarkerOffset(0.8, () -> {/**/}) // target = 40
                                    .UNSTABLE_addTemporalMarkerOffset(1.2, () -> {/**/}) // target = 0
                                    .lineTo(new Vector2d(34, 18))
                                    .addTemporalMarker(() -> {/**/}) // open claw
                                    .addTemporalMarker(() -> {/**/}) // intake on
                                    .lineTo(new Vector2d(-59, 8))
                                    .addTemporalMarker(() -> {/**/}) // intake off
                                    .addTemporalMarker(() -> {/**/}) // close claw
                                    .lineTo(new Vector2d(24, 12))
                                    .addTemporalMarker(() -> {/**/}) // arm up
                                    .splineToConstantHeading(new Vector2d(52, 37), Math.toRadians(0))
                                    .waitSeconds(0.5)
                                    .addTemporalMarker(() -> {/**/}) // open claw
                                    .waitSeconds(0.5)
                                    .addTemporalMarker(() -> {/**/}) // target = 80
                                    .addTemporalMarker(() -> {/**/}) // close claw
                                    .waitSeconds(0.5)
                                    .UNSTABLE_addTemporalMarkerOffset(0.8, () -> {/**/}) // target = 40
                                    .UNSTABLE_addTemporalMarkerOffset(1.2, () -> {/**/}) // target = 0
                                    .lineTo(new Vector2d(36, 45))
                                    .splineToConstantHeading(new Vector2d(60, 60), Math.toRadians(0))
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