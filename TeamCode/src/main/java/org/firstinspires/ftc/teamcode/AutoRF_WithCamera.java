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

@Autonomous(name = "Autonomous RedFront With Camera")
public class AutoRF_WithCamera extends LinearOpMode {
    MecanumTrain bot;

    enum SPIKE_LOC  {
            LEFT,
            CENTER,
            RIGHT
    }
    SPIKE_LOC spikeLoc = SPIKE_LOC.LEFT;
    @Override
    public void runOpMode() throws InterruptedException {
        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        ElapsedTime runtime = new ElapsedTime();
        bot = new MecanumTrain(hardwareMap, runtime);

        bot.initTfodRed(hardwareMap);

        Pose2d startPose = new Pose2d(10.5, -59, Math.toRadians(180));
        bot.odometry.setPoseEstimate(startPose);

        Trajectory findRedProp = bot.odometry.trajectoryBuilder(startPose)
                .lineTo(new Vector2d(14, -50))
                .build();

        TrajectorySequence driveToRSpike = bot.odometry.trajectorySequenceBuilder(findRedProp.end())
                .splineToLinearHeading(new Pose2d(23, -41, Math.toRadians(90)), Math.toRadians(0))
                .addDisplacementMarker(() -> {bot.openClaw();})
                .back(3)
                .addDisplacementMarker(() -> {bot.closeClaw();})
                .splineToSplineHeading(new Pose2d(35, -34.5), Math.toRadians(0))
                .build();

        TrajectorySequence driveToCSpike = bot.odometry.trajectorySequenceBuilder(findRedProp.end())
                .splineToSplineHeading(new Pose2d(19, 27.5, Math.toRadians(120)), Math.toRadians(120))
                .addDisplacementMarker(() -> {bot.openClaw();})
                .back(3)
                .addDisplacementMarker(() -> {bot.closeClaw();})
                .splineToSplineHeading(new Pose2d(30, -36, Math.toRadians(90)), Math.toRadians(0))
                .build();

        waitForStart();

        double x = 0;

        while (opModeIsActive()) {
            while (runtime.seconds() < 3) {
                bot.odometry.followTrajectory(findRedProp);
            }

            List<Recognition> currentRecognitions = bot.tfod.getRecognitions();
            if (bot.tfod.getRecognitions() != null) {
                for (Recognition recognition : currentRecognitions) {
                    x = (recognition.getLeft() + recognition.getRight()) / 2;
                    telemetry.addData("x", x);
                }
            }

            if (x > 400) {
                spikeLoc = SPIKE_LOC.RIGHT;
            }

            if (x > 150 && x < 500) {
                spikeLoc = SPIKE_LOC.CENTER;
            }

            if (spikeLoc == SPIKE_LOC.RIGHT) {
                bot.odometry.followTrajectorySequence(driveToRSpike);
            }

            telemetry.update();

        }
    }
}
