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

@Autonomous(name = "Autonomous RedBack PurpleOnly")
public class AutoRB_PurpleOnly extends LinearOpMode {
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

        Pose2d startPose = new Pose2d(-35, -59, Math.toRadians(180));
        bot.odometry.setPoseEstimate(startPose);

        TrajectorySequence findRedBProp = bot.odometry.trajectorySequenceBuilder(startPose)
                .addTemporalMarker(()-> bot.closeClaw())
                .lineTo(new Vector2d(-40, -53))
                .build();

        TrajectorySequence RBdriveToRSpike = bot.odometry.trajectorySequenceBuilder(startPose)
                .addTemporalMarker(() -> bot.closeClaw())
                .addTemporalMarker(() -> bot.target = 0)
                .setVelConstraint(bot.odometry.SLOW_VEL_CONSTRAINT)
                .splineToSplineHeading(new Pose2d(-48, -33, Math.toRadians(90)), Math.toRadians(90))
                .waitSeconds(0.5)
                .back(4)
                .waitSeconds(0.5)
                .addTemporalMarker(() -> bot.openClaw())
                .back(6)
                .addTemporalMarker(() -> bot.closeClaw())
                .waitSeconds(0.5)
                .back(4)
//                .setAccelConstraint(bot.odometry.SHAKE_ACCEL_CONSTRAINT)
//                .forward(4)
//                .back(4)
//                .resetAccelConstraint()
                .lineTo(new Vector2d(-35, -59))
                .build();

        TrajectorySequence RBdriveToCSpike = bot.odometry.trajectorySequenceBuilder(startPose)
                .addTemporalMarker(() -> bot.closeClaw())
                .addTemporalMarker(() -> bot.target = 0)
                .setVelConstraint(bot.odometry.SLOW_VEL_CONSTRAINT)
                .lineTo(new Vector2d(-40, -50))
                .waitSeconds(1)
                .splineToSplineHeading(new Pose2d(-41, -24, Math.toRadians(45)), Math.toRadians(45))
                .waitSeconds(0.5)
                .addTemporalMarker(() -> bot.openClaw())
                .back(4)
                .addTemporalMarker(() -> bot.closeClaw())
                .waitSeconds(0.5)
                .back(4)
//                .setAccelConstraint(bot.odometry.SHAKE_ACCEL_CONSTRAINT)
//                .forward(4)
//                .back(4)
//                .resetAccelConstraint()
                .splineToSplineHeading(new Pose2d(-40, -55, Math.toRadians(180)), Math.toRadians(180))
                .build();

        TrajectorySequence RBdriveToLSpike = bot.odometry.trajectorySequenceBuilder(startPose)
                .addTemporalMarker(() -> bot.closeClaw())
                .addTemporalMarker(() -> bot.target = 0)
                .setVelConstraint(bot.odometry.SLOW_VEL_CONSTRAINT)
                .lineTo(new Vector2d(-40, -55))
                .waitSeconds(1)
                .lineToSplineHeading(new Pose2d(-39, -37, Math.toRadians(0)))
                .splineToConstantHeading(new Vector2d(-28, -34), Math.toRadians(180))
                .addTemporalMarker(() -> bot.openClaw())
                .waitSeconds(0.5)
                .back(5)
                .addTemporalMarker(() -> bot.closeClaw())
                .waitSeconds(0.5)
                .back(8)
//                .setAccelConstraint(bot.odometry.SHAKE_ACCEL_CONSTRAINT)
//                .forward(4)
//                .back(4)
//                .resetAccelConstraint()
                .lineTo(new Vector2d(-40, -59))
                .build();

        bot.initEocvRed(hardwareMap);
        SPIKE_LOC spikeLoc = SPIKE_LOC.IDLE;
        bot.closeClaw();
        waitForStart();
        runtime.reset();
        double x = 0;

        if (opModeIsActive()) {
            while (opModeIsActive()) {
                while (runtime.seconds() < 1) {
                    bot.target = 0;
                    bot.closeClaw();
                    bot.odometry.followTrajectorySequence(findRedBProp);

                    if (bot.pipelineRed.isPropLeft()){
                        spikeLoc = SPIKE_LOC.LEFT;
                        if (bot.visionPortal.getCameraState() == VisionPortal.CameraState.STREAMING) {
                            bot.visionPortal.close();
                        }
                        break;
                    } else if (bot.pipelineRed.isPropCenter()){
                        spikeLoc = SPIKE_LOC.CENTER;
                        if (bot.visionPortal.getCameraState() == VisionPortal.CameraState.STREAMING) {
                            bot.visionPortal.close();
                        }
                        break;
                    } else {
                        spikeLoc = SPIKE_LOC.RIGHT;
                        if (bot.visionPortal.getCameraState() == VisionPortal.CameraState.STREAMING) {
                            bot.visionPortal.close();
                        }
                    }
                    telemetry.addData("x", bot.pipelineRed.centerX);
                    sleep(20);
                }


                if (bot.visionPortal.getCameraState() == VisionPortal.CameraState.CAMERA_DEVICE_READY) {
                    bot.visionPortal.close();
                }


                switch (spikeLoc) {
                    case RIGHT:
                        telemetry.addLine("RIGHT SPIKE");
                        if (!bot.odometry.isBusy()) {
                            bot.odometry.followTrajectorySequenceAsync(RBdriveToRSpike);
                            spikeLoc = SPIKE_LOC.IDLE;
                            break;
                        }
                    case CENTER:
                        telemetry.addLine("CENTER SPIKE");
                        if (!bot.odometry.isBusy()) {
                            bot.odometry.followTrajectorySequenceAsync(RBdriveToCSpike);
                            spikeLoc = SPIKE_LOC.IDLE;
                            break;
                        }
                    case LEFT:
                        telemetry.addLine("LEFT SPIKE");
                        if (!bot.odometry.isBusy()) {
                            bot.odometry.followTrajectorySequenceAsync(RBdriveToLSpike);
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

