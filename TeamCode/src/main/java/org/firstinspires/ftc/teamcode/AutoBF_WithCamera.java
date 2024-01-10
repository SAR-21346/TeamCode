package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

import java.util.List;

@Autonomous(name = "Autonomous BackFront With Camera")
public class AutoBF_WithCamera extends LinearOpMode {
    MecanumTrain bot;

    enum SPIKE_LOC  {
        LEFT,
        CENTER,
        RIGHT,
        IDLE
    }
    SPIKE_LOC spikeLoc = SPIKE_LOC.LEFT;

    @Override
    public void runOpMode() throws InterruptedException {
        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        ElapsedTime runtime = new ElapsedTime();
        bot = new MecanumTrain(hardwareMap, runtime);

        bot.initTfodBlue(hardwareMap);

        Pose2d startPose = new Pose2d(10.5, 59, Math.toRadians(180));
        bot.odometry.setPoseEstimate(startPose);

        Trajectory findBlueProp = bot.odometry.trajectoryBuilder(startPose)
                .lineTo(new Vector2d(12, -53))
                .build();

        TrajectorySequence driveToLSpike = bot.odometry.trajectorySequenceBuilder(findBlueProp.end())
                .splineToSplineHeading(new Pose2d(23, 46, Math.toRadians(0)), Math.toRadians(0))
                .lineTo(new Vector2d(23, 38))
                .addTemporalMarker(() -> bot.openClaw())
                .waitSeconds(1)
                .back(5)
                .addTemporalMarker(() -> bot.closeClaw())
                .waitSeconds(1)
                .lineTo(new Vector2d(23, 46))
                .lineTo(new Vector2d(25, 46))
                .splineToSplineHeading(new Pose2d(52, 42, Math.toRadians(180)), Math.toRadians(0))
                .build();

        TrajectorySequence driveToCSpike = bot.odometry.trajectorySequenceBuilder(findBlueProp.end())
                .splineToSplineHeading(new Pose2d(20.5, 27.5, Math.toRadians(150)), Math.toRadians(150))
                .addTemporalMarker(() -> bot.openClaw())
                .waitSeconds(1)
                .back(5)
                .addTemporalMarker(() -> bot.closeClaw())
                .waitSeconds(1)
                .addTemporalMarker(6.5, () -> {bot.target = 120;})
                .lineToSplineHeading(new Pose2d(52, 35, Math.toRadians(180)))
                .addTemporalMarker(() -> bot.openClaw())
                .waitSeconds(1)
                .addTemporalMarker(() -> bot.closeClaw())
                .waitSeconds(1)
                .build();

        TrajectorySequence driveToRSpike = bot.odometry.trajectorySequenceBuilder(findBlueProp.end())
                .lineTo(new Vector2d(14, 44))
                .splineToConstantHeading(new Vector2d(4, 33), Math.toRadians(180))
                .waitSeconds(1.5)
                .addTemporalMarker(() -> bot.openClaw())
                .waitSeconds(2)
                .back(5)
                .addTemporalMarker(() -> bot.closeClaw())
                .lineTo(new Vector2d(14, 44))
                .addTemporalMarker(() -> {
                    bot.runLift(1000);
                    bot.target = 120;
                })
                .lineToConstantHeading(new Vector2d(52, 42))
                .waitSeconds(1)
                .addTemporalMarker(() -> bot.openClaw())
                .waitSeconds(1)
                .addTemporalMarker(() -> bot.closeClaw())
                .waitSeconds(2)
                .lineTo(new Vector2d(45, 15))
                .addTemporalMarker(() -> {bot.target = 80;})
                .waitSeconds(0.8)
                .addTemporalMarker(() -> {bot.target = 40;})
                .waitSeconds(0.8)
                .addTemporalMarker(() -> {
                    bot.target = 0;
                    bot.runLift(0);
                })
                .splineToConstantHeading(new Vector2d(60, 12.5), Math.toRadians(0))
                .build();


    }
}