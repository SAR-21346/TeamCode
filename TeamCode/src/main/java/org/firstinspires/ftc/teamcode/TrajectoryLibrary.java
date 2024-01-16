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

    // Camera Trajectories
    Trajectory findRedFProp = bot.odometry.trajectoryBuilder(startPose)
            .lineTo(new Vector2d(12, -53))
            .build();

    Trajectory findRedBProp = bot.odometry.trajectoryBuilder(startPose)
            .lineTo(new Vector2d(-35, -53))
            .build();

    Trajectory findBlueFProp = bot.odometry.trajectoryBuilder(startPose)
            .lineTo(new Vector2d(12, 53))
            .build();

    Trajectory findBlueBProp = bot.odometry.trajectoryBuilder(startPose)
            .lineTo(new Vector2d(-35, 53))
            .build();

    // Red-Front Drive Trajectories
    TrajectorySequence RFdriveToRSpike = bot.odometry.trajectorySequenceBuilder(findRedFProp.end())
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

    TrajectorySequence RFdriveToCSpike = bot.odometry.trajectorySequenceBuilder(findRedFProp.end())
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

    TrajectorySequence RFdriveToLSpike = bot.odometry.trajectorySequenceBuilder(findRedFProp.end())
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

    // Blue-Front Drive Trajectories
    TrajectorySequence BFdriveToLSpike = bot.odometry.trajectorySequenceBuilder(findBlueFProp.end())
            .splineToSplineHeading(new Pose2d(23, 38, Math.toRadians(270)), Math.toRadians(0))
            .waitSeconds(0.5)
            .addTemporalMarker(() -> bot.openClaw())
            .back(5)
            .waitSeconds(0.5)
            .addTemporalMarker(() -> bot.closeClaw())
            .waitSeconds(0.5)
            .lineTo(new Vector2d(25, 43))
            .UNSTABLE_addDisplacementMarkerOffset(6, () -> {bot.target = 130;})
            .splineToSplineHeading(new Pose2d(52, 39, Math.toRadians(180)), Math.toRadians(0))
            .setAccelConstraint(bot.odometry.SHAKE_ACCEL_CONSTRAINT)
            .forward(5)
            .back(5)
            .resetAccelConstraint()
            .waitSeconds(0.5)
            .addTemporalMarker(() -> bot.openClaw())
            .waitSeconds(0.5)
            .addTemporalMarker(() -> {bot.target = 80;})
            .addTemporalMarker(() -> bot.closeClaw())
            .waitSeconds(0.5)
            .lineTo(new Vector2d(45, 18))
            .addTemporalMarker(() -> {bot.target = 40;})
            .waitSeconds(0.3)
            .addTemporalMarker(() -> {bot.target = 0;})
            .lineTo(new Vector2d(60, 7))
            .build();

    TrajectorySequence BFdriveToCSpike = bot.odometry.trajectorySequenceBuilder(findBlueFProp.end())
            .lineTo(new Vector2d(14, 40))
            .splineToSplineHeading(new Pose2d(19, 27, Math.toRadians(230)), Math.toRadians(270))
            .waitSeconds(0.5)
            .addTemporalMarker(() -> bot.openClaw())
            .back(5)
            .addTemporalMarker(() -> bot.closeClaw())
            .waitSeconds(0.5)
            .UNSTABLE_addDisplacementMarkerOffset(6, () -> {
                bot.target = 130;
            })
            .lineToSplineHeading(new Pose2d(52, 33, Math.toRadians(180)))
            .setAccelConstraint(bot.odometry.SHAKE_ACCEL_CONSTRAINT)
            .forward(5)
            .back(5)
            .resetAccelConstraint()
            .waitSeconds(0.5)
            .addTemporalMarker(() -> bot.openClaw())
            .waitSeconds(0.5)
            .addTemporalMarker(() -> {
                bot.target = 80;
            })
            .addTemporalMarker(() -> bot.closeClaw())
            .waitSeconds(0.5)
            .lineTo(new Vector2d(45, 15))
            .addTemporalMarker(() -> {
                bot.target = 40;
            })
            .waitSeconds(0.3)
            .addTemporalMarker(() -> {
                bot.target = 0;
            })
            .lineTo(new Vector2d(60, 7))
            .build();

    TrajectorySequence BFdriveToRSpike = bot.odometry.trajectorySequenceBuilder(findBlueFProp.end())
            .lineToSplineHeading(new Pose2d(15, 45, Math.toRadians(180)))
            .splineToConstantHeading(new Vector2d(5, 29), Math.toRadians(180))
            .waitSeconds(.5)
            .addTemporalMarker(() -> bot.openClaw())
            .addTemporalMarker(() -> {bot.target = 0;})
            .back(4)
            .addTemporalMarker(() -> bot.closeClaw())
            .waitSeconds(0.5)
            .UNSTABLE_addDisplacementMarkerOffset(12, () -> {
                bot.target = 130;
            })
            .lineToConstantHeading(new Vector2d(52, 27))
            .setAccelConstraint(bot.odometry.SHAKE_ACCEL_CONSTRAINT)
            .forward(5)
            .back(5)
            .resetAccelConstraint()
            .waitSeconds(.5)
            .addTemporalMarker(() -> bot.openClaw())
            .waitSeconds(.8)
            .addTemporalMarker(() -> {
                bot.target = 80;
            })
            .addTemporalMarker(() -> bot.closeClaw())
            .waitSeconds(1)
            .lineTo(new Vector2d(45, 15))
            .addTemporalMarker(() -> {
                bot.target = 40;
            })
            .waitSeconds(.4)
            .addTemporalMarker(() -> {
                bot.target = 0;
            })
            .lineTo(new Vector2d(60, 7))
            .build();

    // Red-Back Drive Trajectories
    TrajectorySequence RBdriveToLSpike = bot.odometry.trajectorySequenceBuilder(findRedBProp.end())
            .lineToSplineHeading(new Pose2d(-46.5, -35, Math.toRadians(90)))
            .waitSeconds(0.5)
            .addTemporalMarker(() -> bot.openClaw())
            .back(6)
            .addTemporalMarker(() -> bot.closeClaw())
            .waitSeconds(0.5)
            .strafeRight(3)
            .splineToConstantHeading(new Vector2d(-34, -35), Math.toRadians(90))
            .splineToConstantHeading(new Vector2d(-34,-20), Math.toRadians(90))
            .splineToConstantHeading(new Vector2d(5, -13), Math.toRadians(0))
            .UNSTABLE_addDisplacementMarkerOffset(24, () -> {bot.target = 130;})
            .splineToSplineHeading(new Pose2d(52, -24, Math.toRadians(180)), Math.toRadians(0))
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
            .lineTo(new Vector2d(36, -45))
            .addTemporalMarker(() -> {bot.target = 40;})
            .waitSeconds(0.4)
            .addTemporalMarker(() -> {bot.target = 0;})
            .splineToConstantHeading(new Vector2d(60, -60), Math.toRadians(0))
            .build();

    TrajectorySequence RBdriveToCSpike = bot.odometry.trajectorySequenceBuilder(findRedBProp.end())
            .splineToSplineHeading(new Pose2d(-41, -24, Math.toRadians(45)), Math.toRadians(45))
            .waitSeconds(0.5)
            .addTemporalMarker(() -> bot.openClaw())
            .back(6)
            .addTemporalMarker(() -> bot.closeClaw())
            .waitSeconds(0.5)
            .strafeLeft(3)
            .splineToSplineHeading(new Pose2d(-47, -9.5, Math.toRadians(90)), Math.toRadians(0))
            .splineToConstantHeading(new Vector2d(5, -13), Math.toRadians(0))
            .UNSTABLE_addDisplacementMarkerOffset(24, () -> {bot.target = 130;})
            .splineToSplineHeading(new Pose2d(52, -33, Math.toRadians(180)), Math.toRadians(0))
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
            .lineTo(new Vector2d(36, -45))
            .addTemporalMarker(() -> {bot.target = 40;})
            .waitSeconds(0.4)
            .addTemporalMarker(() -> {bot.target = 0;})
            .splineToConstantHeading(new Vector2d(60, -60), Math.toRadians(0))
            .build();

    TrajectorySequence RBdriveToRSpike = bot.odometry.trajectorySequenceBuilder(findRedBProp.end())
            .lineToSplineHeading(new Pose2d(-38, -40, Math.toRadians(0)))
            .splineToConstantHeading(new Vector2d(-28, -34), Math.toRadians(0))
            .addTemporalMarker(() -> bot.openClaw())
            .waitSeconds(0.5)
            .back(5)
            .waitSeconds(0.5)
            .addTemporalMarker(() -> bot.closeClaw())
            .waitSeconds(0.5)
            .strafeLeft(5)
            .splineToConstantHeading(new Vector2d(-34, -11), Math.toRadians(0))
            .UNSTABLE_addDisplacementMarkerOffset(48, () -> {bot.target = 130;})
            .lineToConstantHeading(new Vector2d(10, -11))
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
            .lineTo(new Vector2d(36,-45))
            .addTemporalMarker(() -> {bot.target = 40;})
            .waitSeconds(0.4)
            .addTemporalMarker(() -> {bot.target = 0;})
            .splineToConstantHeading(new Vector2d(60, -60), Math.toRadians(0))
            .build();

    // Blue-Back Drive Trajectories
    TrajectorySequence BBdriveToLSpike = bot.odometry.trajectorySequenceBuilder(findBlueBProp.end())
            .build();

    TrajectorySequence BBdriveToCSpike = bot.odometry.trajectorySequenceBuilder(findBlueBProp.end())
            .build();

    TrajectorySequence BBdriveToRSpike = bot.odometry.trajectorySequenceBuilder(findBlueBProp.end())
            .build();

}
