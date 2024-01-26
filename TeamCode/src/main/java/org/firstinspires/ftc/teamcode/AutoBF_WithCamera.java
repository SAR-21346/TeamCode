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
    public void runOpMode(){
        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        ElapsedTime runtime = new ElapsedTime();
        bot = new MecanumTrain(hardwareMap, runtime);

        Pose2d startPose = new Pose2d(9.5, 59, Math.toRadians(0));
        bot.odometry.setPoseEstimate(startPose);

        // Create TrajectoryLibrary to de-clutter OpMode
        //TrajectoryLibrary trajLib = new TrajectoryLibrary(bot, startPose);

        Trajectory findBlueFProp = bot.odometry.trajectoryBuilder(startPose)
                .lineTo(new Vector2d(12, 53))
                .build();


        TrajectorySequence BFdriveToLSpike = bot.odometry.trajectorySequenceBuilder(findBlueFProp.end())
                .splineToSplineHeading(new Pose2d(23, 38, Math.toRadians(270)), Math.toRadians(0))
                .waitSeconds(0.5)
                .addTemporalMarker(() -> bot.openClaw())
                .back(3)
                .waitSeconds(0.5)
                .addTemporalMarker(() -> bot.closeClaw())
                .waitSeconds(0.5)
                .lineTo(new Vector2d(25, 43))
                .UNSTABLE_addDisplacementMarkerOffset(6, () -> {bot.target = 130;})
                .splineToSplineHeading(new Pose2d(52, 35, Math.toRadians(180)), Math.toRadians(0))
                .setAccelConstraint(bot.odometry.SHAKE_ACCEL_CONSTRAINT)
                .forward(4)
                .back(4)
                .resetAccelConstraint()
                .waitSeconds(0.5)
                .addTemporalMarker(() -> bot.openClaw())
                .waitSeconds(0.5)
                .addTemporalMarker(() -> {bot.target = 80;})
                .addTemporalMarker(() -> bot.closeClaw())
                .waitSeconds(0.5)
                .lineTo(new Vector2d(45, 15))
                .addTemporalMarker(() -> {bot.target = 40;})
                .waitSeconds(0.3)
                .addTemporalMarker(() -> {bot.target = 0;})
                .lineTo(new Vector2d(60, 8))
                .build();

        TrajectorySequence BFdriveToCSpike = bot.odometry.trajectorySequenceBuilder(findBlueFProp.end())
                .lineTo(new Vector2d(14, 40))
                .splineToSplineHeading(new Pose2d(18, 24, Math.toRadians(230)), Math.toRadians(270))
                .waitSeconds(0.5)
                .addTemporalMarker(() -> bot.openClaw())
                .back(4)
                .addTemporalMarker(() -> bot.closeClaw())
                .waitSeconds(0.5)
                .UNSTABLE_addDisplacementMarkerOffset(10, () -> {
                    bot.target = 130;
                })
                .lineToSplineHeading(new Pose2d(52, 31, Math.toRadians(180)))
                .setAccelConstraint(bot.odometry.SHAKE_ACCEL_CONSTRAINT)
                .forward(4)
                .back(4)
                .resetAccelConstraint()
                .waitSeconds(0.5)
                .addTemporalMarker(() -> bot.openClaw())
                .waitSeconds(0.5)
                .addTemporalMarker(() -> {
                    bot.target = 80;
                })
                .addTemporalMarker(() -> bot.closeClaw())
                .waitSeconds(0.5)
                .lineTo(new Vector2d(42, 10))
                .addTemporalMarker(() -> {
                    bot.target = 40;
                })
                .waitSeconds(0.2)
                .addTemporalMarker(() -> {
                    bot.target = 0;
                })
                .lineTo(new Vector2d(60, 8))
                .build();

        TrajectorySequence BFdriveToRSpike = bot.odometry.trajectorySequenceBuilder(findBlueFProp.end())
                .lineToSplineHeading(new Pose2d(15, 45, Math.toRadians(180)))
                .splineToConstantHeading(new Vector2d(4, 29), Math.toRadians(180))
                .waitSeconds(.5)
                .addTemporalMarker(() -> bot.openClaw())
                .back(5)
                .addTemporalMarker(() -> bot.closeClaw())
                .waitSeconds(0.5)
                .UNSTABLE_addDisplacementMarkerOffset(12, () -> {
                    bot.target = 130;
                })
                .lineToConstantHeading(new Vector2d(52, 27))
                .setAccelConstraint(bot.odometry.SHAKE_ACCEL_CONSTRAINT)
                .forward(6)
                .back(6)
                .resetAccelConstraint()
                .waitSeconds(.5)
                .addTemporalMarker(() -> bot.openClaw())
                .waitSeconds(.8)
                .addTemporalMarker(() -> {
                    bot.target = 80;
                })
                .addTemporalMarker(() -> bot.closeClaw())
                .waitSeconds(1)
                .lineTo(new Vector2d(42, 13))
                .addTemporalMarker(() -> {
                    bot.target = 40;
                })
                .waitSeconds(.4)
                .addTemporalMarker(() -> {
                    bot.target = 0;
                })
                .lineTo(new Vector2d(60, 8))
                .build();

        bot.initEocvBlue(hardwareMap);
        SPIKE_LOC spikeLoc = SPIKE_LOC.IDLE;
        waitForStart();
        bot.closeClaw();
        runtime.reset();
        double x = 0;

        if (opModeIsActive()) {
            while (opModeIsActive()) {
                while (runtime.seconds() < 3) {
                    bot.closeClaw();
                    bot.odometry.followTrajectory(findBlueFProp);

                    if (bot.pipeline.isPropRight()){
                        spikeLoc = SPIKE_LOC.RIGHT;
                        if (bot.visionPortal.getCameraState() == VisionPortal.CameraState.STREAMING) {
                            bot.visionPortal.close();
                        }
                        break;
                    } else if (bot.pipeline.isPropCenter()){
                        spikeLoc = SPIKE_LOC.CENTER;
                        if (bot.visionPortal.getCameraState() == VisionPortal.CameraState.STREAMING) {
                            bot.visionPortal.close();
                        }
                        break;
                    } else if (!(bot.pipeline.isPropCenter() && bot.pipeline.isPropRight())) {
                        spikeLoc = SPIKE_LOC.LEFT;
                    }
                    telemetry.addData("x", bot.pipeline.centerX);

                    //Try this first
                    sleep(20);
                }


                if (bot.visionPortal.getCameraState() == VisionPortal.CameraState.CAMERA_DEVICE_READY) {
                    bot.visionPortal.close();
                }


                switch (spikeLoc) {
                    case RIGHT:
                        telemetry.addLine("RIGHT SPIKE");
                        if (!bot.odometry.isBusy()) {
                            bot.odometry.followTrajectorySequenceAsync(BFdriveToRSpike);
                            spikeLoc = SPIKE_LOC.IDLE;
                            break;
                        }
                    case CENTER:
                        telemetry.addLine("CENTER SPIKE");
                        if (!bot.odometry.isBusy()) {
                            bot.odometry.followTrajectorySequenceAsync(BFdriveToCSpike);
                            spikeLoc = SPIKE_LOC.IDLE;
                            break;
                        }
                    case LEFT:
                        telemetry.addLine("LEFT SPIKE");
                        if (!bot.odometry.isBusy()) {
                            bot.odometry.followTrajectorySequenceAsync(BFdriveToLSpike);
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
}