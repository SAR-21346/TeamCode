package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;

import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

public class TrajectoryLibrary {
    private MecanumTrain bot;
    private Pose2d startPose;
    public TrajectoryLibrary (MecanumTrain botObj, Pose2d startPoseX) {
        bot = botObj;
        startPose = startPoseX;
    }

    Trajectory findRedProp = bot.odometry.trajectoryBuilder(startPose)
            .lineTo(new Vector2d(12, -53))
            .build();

    TrajectorySequence driveToRSpike = bot.odometry.trajectorySequenceBuilder(findRedProp.end())
            .splineToSplineHeading(new Pose2d(23, -38, Math.toRadians(90)), Math.toRadians(0))
            .lineTo(new Vector2d(23, -35))
            .addTemporalMarker(() -> bot.openClaw())
            .waitSeconds(0.5)
            .back(5)
            .waitSeconds(0.5)
            .addTemporalMarker(() -> bot.closeClaw())
            .waitSeconds(0.5)
            .lineTo(new Vector2d(28, -39))
            .UNSTABLE_addDisplacementMarkerOffset(6, () -> {bot.target = 130;})
            .splineToSplineHeading(new Pose2d(52, -39, Math.toRadians(180)), Math.toRadians(0))
            .setAccelConstraint(bot.odometry.SHAKE_ACCEL_CONSTRAINT)
            .forward(6)
            .back(6)
            .resetAccelConstraint()
            .waitSeconds(0.5)
            .addTemporalMarker(() -> bot.openClaw())
            .waitSeconds(0.5)
            .addTemporalMarker(() -> {bot.target = 80;})
            .addTemporalMarker(() -> bot.closeClaw())
            .waitSeconds(1)
            .lineTo(new Vector2d(45, -18))
            .addTemporalMarker(() -> {bot.target = 40;})
            .waitSeconds(0.4)
            .addTemporalMarker(() -> {bot.target = 0;})
            .lineTo(new Vector2d(60, -5))
            .build();

    TrajectorySequence driveToCSpike = bot.odometry.trajectorySequenceBuilder(findRedProp.end())
            .lineTo(new Vector2d(14, -40))
            .splineToSplineHeading(new Pose2d(19, -23, Math.toRadians(150)), Math.toRadians(90))
            .waitSeconds(0.5)
            .addTemporalMarker(() -> bot.openClaw())
            .back(5)
            .addTemporalMarker(() -> bot.closeClaw())
            .waitSeconds(0.5)
            .UNSTABLE_addDisplacementMarkerOffset(5, () -> {bot.target = 130;})
            .lineToSplineHeading(new Pose2d(52, -29, Math.toRadians(180)))
            .setAccelConstraint(bot.odometry.SHAKE_ACCEL_CONSTRAINT)
            .forward(6)
            .back(6)
            .resetAccelConstraint()
            .waitSeconds(0.5)
            .addTemporalMarker(() -> bot.openClaw())
            .waitSeconds(0.8)
            .addTemporalMarker(() -> {bot.target = 80;})
            .addTemporalMarker(() -> bot.closeClaw())
            .waitSeconds(1)
            .lineTo(new Vector2d(34, -18))
            .addTemporalMarker(() -> {bot.target = 40;})
            .waitSeconds(0.4)
            .addTemporalMarker(() -> {bot.target = 0;})
            .splineToConstantHeading(new Vector2d(60, -5), Math.toRadians(0))
            .build();

    TrajectorySequence driveToLSpike = bot.odometry.trajectorySequenceBuilder(findRedProp.end())
            .lineTo(new Vector2d(14, -40))
            .splineToConstantHeading(new Vector2d(4, -29), Math.toRadians(180))
            .waitSeconds(0.5)
            .addTemporalMarker(() -> bot.openClaw())
            .back(6)
            .addTemporalMarker(() -> bot.closeClaw())
            .waitSeconds(0.5)
            .addTemporalMarker(5.5, () -> {bot.target = 130;})
            .lineToConstantHeading(new Vector2d(52, -24))
            .setAccelConstraint(bot.odometry.SHAKE_ACCEL_CONSTRAINT)
            .forward(5)
            .back(5)
            .resetAccelConstraint()
            .waitSeconds(0.5)
            .addTemporalMarker(() -> bot.openClaw())
            .waitSeconds(0.8)
            .addTemporalMarker(() -> {bot.target = 80;})
            .addTemporalMarker(() -> bot.closeClaw())
            .waitSeconds(1)
            .lineTo(new Vector2d(34, -18))
            .addTemporalMarker(() -> {bot.target = 40;})
            .waitSeconds(0.4)
            .addTemporalMarker(() -> {bot.target = 0;})
            .splineToConstantHeading(new Vector2d(60, -5), Math.toRadians(0))
            .build();
}
