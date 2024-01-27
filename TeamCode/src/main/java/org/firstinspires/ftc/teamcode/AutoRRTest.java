package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;


@Autonomous(name = "AutoRRTest")
@Disabled
public class AutoRRTest extends LinearOpMode {
    MecanumTrain bot;
    ElapsedTime runtime;
    @Override
    public void runOpMode() throws InterruptedException {
        runtime = new ElapsedTime();
        bot = new MecanumTrain(hardwareMap, runtime);

        Pose2d startPose = new Pose2d(10.5, 61.0 , Math.toRadians(270));

        bot.odometry.setPoseEstimate(startPose);
        Trajectory test = bot.odometry.trajectoryBuilder(startPose)
                .splineTo(new Vector2d(0, 35.0), Math.toRadians(180))
                .splineTo(new Vector2d(-56.0, 35.0), Math.toRadians(180))
                .splineToSplineHeading(new Pose2d(-52, 35), Math.toRadians(180))
                .splineToSplineHeading(new Pose2d(-26, 12), Math.toRadians(0))
                .splineTo(new Vector2d(26, 12), Math.toRadians(0))
                .splineTo(new Vector2d(10.5, 40), Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(10.5, 60.0), Math.toRadians(90))
                .build();

        waitForStart();
        if (isStopRequested()) return;

        bot.odometry.followTrajectory(test);
    }
}
