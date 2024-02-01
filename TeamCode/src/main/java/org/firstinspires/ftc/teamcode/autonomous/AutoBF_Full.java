package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.MecanumTrain;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.vision.VisionPortal;

@Autonomous(name = "Autonomous BlueFront Full")
public class AutoBF_Full extends LinearOpMode {
    MecanumTrain bot;
    enum SPIKE_LOC {
        LEFT,
        CENTER,
        RIGHT,
        IDLE
    }

    @Override
    public void runOpMode() throws InterruptedException{
        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        ElapsedTime runtime = new ElapsedTime();
        bot = new MecanumTrain(hardwareMap, runtime);

        Pose2d startPose = new Pose2d(10.5, 59, Math.toRadians(0));
        bot.odometry.setPoseEstimate(startPose);

        TrajectorySequence findBlueFProp = bot.odometry.trajectorySequenceBuilder(startPose)
                .addTemporalMarker(() -> bot.closeClaw())
                .lineTo(new Vector2d(15, 53))
                .build();

        TrajectorySequence BFdriveToLSpike = bot.odometry.trajectorySequenceBuilder(findBlueFProp.end())
                .splineToSplineHeading(new Pose2d(23, 38, Math.toRadians(270)), Math.toRadians(0))
                .waitSeconds(0.5)
                .addTemporalMarker(() -> bot.openClaw())
                .back(3)
                .waitSeconds(0.5)
                .addTemporalMarker(() -> bot.closeClaw())
                .waitSeconds(0.5)
                .back(9)
                .UNSTABLE_addDisplacementMarkerOffset(6, () -> {bot.target = 130;})
//                .setAccelConstraint(bot.odometry.SHAKE_ACCEL_CONSTRAINT)
//                .forward(4)
//                .back(4)
//                .resetAccelConstraint()
                .lineTo(new Vector2d(25, 43))
                .splineToSplineHeading(new Pose2d(53, 35, Math.toRadians(180)), Math.toRadians(0))
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
                .lineTo(new Vector2d(60, 5))
                .build();

        TrajectorySequence BFdriveToCSpike = bot.odometry.trajectorySequenceBuilder(findBlueFProp.end())
                .lineTo(new Vector2d(14, 40))
                .splineToSplineHeading(new Pose2d(18, 24, Math.toRadians(230)), Math.toRadians(270))
                .waitSeconds(0.5)
                .addTemporalMarker(() -> bot.openClaw())
                .back(4)
                .addTemporalMarker(() -> bot.closeClaw())
                .waitSeconds(0.5)
                .back(6)
                .strafeLeft(3)
//                .setAccelConstraint(bot.odometry.SHAKE_ACCEL_CONSTRAINT)
//                .forward(4)
//                .back(4)
//                .resetAccelConstraint()
                .UNSTABLE_addDisplacementMarkerOffset(10, () -> {
                    bot.target = 130;
                })
                .lineToSplineHeading(new Pose2d(53, 31, Math.toRadians(180)))
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
                .lineTo(new Vector2d(60, 5))
                .build();

        TrajectorySequence BFdriveToRSpike = bot.odometry.trajectorySequenceBuilder(findBlueFProp.end())
                .lineToSplineHeading(new Pose2d(15, 45, Math.toRadians(180)))
                .splineToConstantHeading(new Vector2d(4, 29), Math.toRadians(180))
                .waitSeconds(.5)
                .addTemporalMarker(() -> bot.openClaw())
                .back(5)
                .addTemporalMarker(() -> bot.closeClaw())
                .waitSeconds(0.5)
                .back(8)
//                .setAccelConstraint(bot.odometry.SHAKE_ACCEL_CONSTRAINT)
//                .forward(6)
//                .back(6)
//                .resetAccelConstraint()
                .UNSTABLE_addDisplacementMarkerOffset(12, () -> {
                    bot.target = 130;
                })
                .lineToConstantHeading(new Vector2d(53, 27))
                .waitSeconds(.5)
                .addTemporalMarker(() -> bot.openClaw())
                .waitSeconds(.8)
                .addTemporalMarker(() -> {
                    bot.target = 80;
                })
                .addTemporalMarker(() -> bot.closeClaw())
                .waitSeconds(1)
                .lineTo(new Vector2d(42, 13))
                .waitSeconds(1)
                .addTemporalMarker(() -> bot.openClaw())
                .waitSeconds(1)
                .addTemporalMarker(() -> {
                    bot.target = 80;
                })
                .addTemporalMarker(() -> bot.closeClaw())
                .waitSeconds(1)
                .setVelConstraint(bot.odometry.SLOW_VEL_CONSTRAINT)
                .UNSTABLE_addTemporalMarkerOffset(0.8, () -> bot.target = 40)
                .UNSTABLE_addTemporalMarkerOffset(1.2, () -> bot.target = 0)
                .lineTo(new Vector2d(34, 18))
                .splineToConstantHeading(new Vector2d(60, 5), Math.toRadians(0))
                .build();

        bot.initEocvBlue(hardwareMap);
        SPIKE_LOC spikeLoc = SPIKE_LOC.IDLE;
        waitForStart();
        bot.closeClaw();
        runtime.reset();
        double x = 0;

        if (opModeIsActive()) {
            while (opModeIsActive()) {
                while (runtime.seconds() < 2) {
                    bot.closeClaw();
                    bot.odometry.followTrajectorySequence(findBlueFProp);

                    if (bot.pipeline.isPropRight()) {
                        spikeLoc = SPIKE_LOC.RIGHT;
                        telemetry.addData("x", bot.pipelineRed.centerX);
                        if (bot.visionPortal.getCameraState() == VisionPortal.CameraState.STREAMING) {
                            bot.visionPortal.close();
                        }
                        break;
                    } else if (bot.pipeline.isPropCenter()) {
                        spikeLoc = SPIKE_LOC.CENTER;
                        telemetry.addData("x", bot.pipelineRed.centerX);
                        if (bot.visionPortal.getCameraState() == VisionPortal.CameraState.STREAMING) {
                            bot.visionPortal.close();
                        }
                        break;
                    } else {
                        spikeLoc = SPIKE_LOC.LEFT;
                        telemetry.addData("x", bot.pipelineRed.centerX);
                        if (bot.visionPortal.getCameraState() == VisionPortal.CameraState.STREAMING) {
                            bot.visionPortal.close();
                        }
                    }
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