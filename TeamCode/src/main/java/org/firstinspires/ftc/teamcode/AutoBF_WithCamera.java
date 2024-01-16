package org.firstinspires.ftc.teamcode;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.vision.VisionPortal;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.List;

@Autonomous(name = "Autonomous BlueFront With Camera")
public class AutoBF_WithCamera extends LinearOpMode {
    MecanumTrain bot;

    enum SPIKE_LOC {
        LEFT,
        CENTER,
        RIGHT,
        IDLE
    }

    @Override
    public void runOpMode() throws InterruptedException {
        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        ElapsedTime runtime = new ElapsedTime();
        bot = new MecanumTrain(hardwareMap, runtime);

        bot.initEocvBlue(hardwareMap);

        // TODO: move back to y=59
        Pose2d startPose = new Pose2d(10.5, 59, Math.toRadians(0));
        bot.odometry.setPoseEstimate(startPose);

        Trajectory findBlueProp = bot.odometry.trajectoryBuilder(startPose)
                .lineTo(new Vector2d(12, 53))
                .build();

        TrajectorySequence driveToLSpike = bot.odometry.trajectorySequenceBuilder(findBlueProp.end())
                .splineToSplineHeading(new Pose2d(23, 38, Math.toRadians(270)), Math.toRadians(0))
                .waitSeconds(0.5)
                .addTemporalMarker(() -> bot.openClaw())
                .back(5)
                .waitSeconds(0.5)
                .addTemporalMarker(() -> bot.closeClaw())
                .waitSeconds(0.5)
                .lineTo(new Vector2d(25, 43))
                .UNSTABLE_addDisplacementMarkerOffset(6, () -> {
                    bot.target = 130;
                })
                .splineToSplineHeading(new Pose2d(52, 39, Math.toRadians(180)), Math.toRadians(0))
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
                .lineTo(new Vector2d(45, 18))
                .addTemporalMarker(() -> {
                    bot.target = 40;
                })
                .waitSeconds(0.3)
                .addTemporalMarker(() -> {
                    bot.target = 0;
                })
                .lineTo(new Vector2d(60, 7))
                .build();

        TrajectorySequence driveToCSpike = bot.odometry.trajectorySequenceBuilder(findBlueProp.end())
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

        TrajectorySequence driveToRSpike = bot.odometry.trajectorySequenceBuilder(findBlueProp.end())
                .lineToSplineHeading(new Pose2d(15, 45, Math.toRadians(180)))
                .splineToConstantHeading(new Vector2d(5, 29), Math.toRadians(180))
                .waitSeconds(.5)
                .addTemporalMarker(() -> bot.openClaw())
                .addTemporalMarker(() -> {
                    bot.target = 0;
                })
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

        bot.closeClaw();
        SPIKE_LOC spikeLoc = SPIKE_LOC.IDLE;
        waitForStart();
        double x = 0;

        while (opModeIsActive()) {
            while (runtime.seconds() < 4) {
                bot.closeClaw();
                bot.odometry.followTrajectory(findBlueProp);

                if (bot.pipeline.isRight()){
                    spikeLoc = SPIKE_LOC.RIGHT;
                } else if (bot.pipeline.isCenter()){
                    spikeLoc = SPIKE_LOC.CENTER;
                } else if (bot.pipeline.isLeft()){
                    spikeLoc = SPIKE_LOC.LEFT;
                }
                telemetry.addData("x", x);
            }

            switch (spikeLoc) {
                case RIGHT:
                    telemetry.addLine("RIGHT SPIKE");
                    if (!bot.odometry.isBusy()) {
                        bot.odometry.followTrajectorySequenceAsync(driveToRSpike);
                        spikeLoc = SPIKE_LOC.IDLE;
                        break;
                    }
                case CENTER:
                    telemetry.addLine("CENTER SPIKE");
                    if (!bot.odometry.isBusy()) {
                        bot.odometry.followTrajectorySequenceAsync(driveToCSpike);
                        spikeLoc = SPIKE_LOC.IDLE;
                        break;
                    }
                case LEFT:
                    telemetry.addLine("LEFT SPIKE");
                    if (!bot.odometry.isBusy()) {
                        bot.odometry.followTrajectorySequenceAsync(driveToLSpike);
                        spikeLoc = SPIKE_LOC.IDLE;
                        break;
                    }
                case IDLE:
                    break;
            }
            bot.updateArmPID(bot.outMotor.getCurrentPosition());
            bot.odometry.update();
            telemetry.update();

        }

        }
    }