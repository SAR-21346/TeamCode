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

@Autonomous(name = "Autonomous RedBack Full")
public class AutoRB_Full extends LinearOpMode {
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

        Trajectory findRedBProp = bot.odometry.trajectoryBuilder(startPose)
                .lineTo(new Vector2d(-44, -53))
                .build();

        TrajectorySequence RBdriveToLSpike = bot.odometry.trajectorySequenceBuilder(startPose)
                .addTemporalMarker(() -> bot.closeClaw())
                .addTemporalMarker(() -> bot.target = 0)
                .setVelConstraint(bot.odometry.SLOW_VEL_CONSTRAINT)
                .lineTo(new Vector2d(-40, -55))
                .waitSeconds(0.5)
                .lineToSplineHeading(new Pose2d(-47, -35, Math.toRadians(90)))
                .waitSeconds(0.5)
                .addTemporalMarker(() -> bot.openClaw())
                .waitSeconds(0.5)
                .back(5)
                .addTemporalMarker(() -> bot.closeClaw())
                .waitSeconds(0.5)
                .back(8)
                .lineTo(new Vector2d(-40, -55))
                .UNSTABLE_addDisplacementMarkerOffset(81, () -> bot.target = 140)
                .lineTo(new Vector2d(14, -55))
                .setAccelConstraint(bot.odometry.SHAKE_ACCEL_CONSTRAINT)
                .forward(6)
                .waitSeconds(0.5)
                //.back(6)
                .resetAccelConstraint()
                .splineToSplineHeading(new Pose2d(52, -24, Math.toRadians(180)), Math.toRadians(0))
                .waitSeconds(0.5)
                .addTemporalMarker(() -> bot.openClaw())
                .waitSeconds(0.5)
                .addTemporalMarker(() -> {
                    bot.target = 80;
                })
                .addTemporalMarker(() -> bot.closeClaw())
                .waitSeconds(1)
                .UNSTABLE_addTemporalMarkerOffset(0.8, () -> bot.target = 40)
                .UNSTABLE_addTemporalMarkerOffset(1.2, () -> bot.target = 0)
                .lineTo(new Vector2d(36, -45))
                .splineToConstantHeading(new Vector2d(60, -60), Math.toRadians(0))
                .build();

        TrajectorySequence RBdriveToCSpike = bot.odometry.trajectorySequenceBuilder(startPose)
                .addTemporalMarker(() -> bot.closeClaw())
                .addTemporalMarker(() -> bot.target = 0)
                .setVelConstraint(bot.odometry.SLOW_VEL_CONSTRAINT)
                .lineTo(new Vector2d(-40, -55))
                .waitSeconds(0.5)
                .splineToSplineHeading(new Pose2d(-41, -24, Math.toRadians(45)), Math.toRadians(45))
                .waitSeconds(0.5)
                .addTemporalMarker(() -> bot.openClaw())
                .waitSeconds(0.5)
                .back(5)
                .addTemporalMarker(() -> bot.closeClaw())
                .waitSeconds(0.5)
                .back(8)
                .lineTo(new Vector2d(-40, -55))
                .turn(Math.toRadians(-45))
                .UNSTABLE_addDisplacementMarkerOffset(81, () -> bot.target = 140)
                .lineTo(new Vector2d(14, -55))
                .waitSeconds(0.5)
                .setAccelConstraint(bot.odometry.SHAKE_ACCEL_CONSTRAINT)
                .forward(6)
                .waitSeconds(0.5)
                .resetAccelConstraint()
                .splineToSplineHeading(new Pose2d(52, -28, Math.toRadians(180)), Math.toRadians(0))
                .addTemporalMarker(() -> bot.openClaw())
                .waitSeconds(0.5)
                .addTemporalMarker(() -> {
                    bot.target = 80;
                })
                .addTemporalMarker(() -> bot.closeClaw())
                .waitSeconds(1)
                .UNSTABLE_addTemporalMarkerOffset(0.8, () -> bot.target = 40)
                .UNSTABLE_addTemporalMarkerOffset(1.2, () -> bot.target = 0)
                .lineTo(new Vector2d(36, -45))
                .splineToConstantHeading(new Vector2d(60, -60), Math.toRadians(0))
                .build();

        TrajectorySequence RBdriveToRSpike = bot.odometry.trajectorySequenceBuilder(startPose)
                .addTemporalMarker(() -> bot.closeClaw())
                .addTemporalMarker(() -> bot.target = 0)
                .setVelConstraint(bot.odometry.SLOW_VEL_CONSTRAINT)
                .lineTo(new Vector2d(-40, -55))
                .waitSeconds(0.5)
                .lineToSplineHeading(new Pose2d(-43, -40, Math.toRadians(0)))
                .splineToConstantHeading(new Vector2d(-28, -34), Math.toRadians(0))
                .addTemporalMarker(() -> bot.openClaw())
                .waitSeconds(0.5)
                .back(5)
                .addTemporalMarker(() -> bot.closeClaw())
                .waitSeconds(0.5)
                .back(8)
                .lineTo(new Vector2d(-40, -55))
                .UNSTABLE_addDisplacementMarkerOffset(81, () -> bot.target = 140)
                .lineTo(new Vector2d(14, -55))
                .waitSeconds(0.5)
                .setAccelConstraint(bot.odometry.SHAKE_ACCEL_CONSTRAINT)
                .forward(5)
                .waitSeconds(0.5)
                .resetConstraints()
                .splineToSplineHeading(new Pose2d(52, -33, Math.toRadians(180)), Math.toRadians(0))
                .waitSeconds(0.5)
                .addTemporalMarker(() -> bot.openClaw())
                .waitSeconds(0.5)
                .addTemporalMarker(() -> {
                    bot.target = 80;
                })
                .addTemporalMarker(() -> bot.closeClaw())
                .waitSeconds(1)
                .lineTo(new Vector2d(36, -45))
                .UNSTABLE_addTemporalMarkerOffset(0.8, () -> bot.target = 40)
                .UNSTABLE_addTemporalMarkerOffset(1.2, () -> bot.target = 0)
                .splineToConstantHeading(new Vector2d(60, -60), Math.toRadians(0))
                .build();

        bot.initEocvRed(hardwareMap);
        SPIKE_LOC spikeLoc = SPIKE_LOC.IDLE;
        waitForStart();
        bot.closeClaw();
        runtime.reset();
        double x = 0;

        if (opModeIsActive()) {
            while (opModeIsActive()) {
                while (runtime.seconds() < 1) {
                    bot.closeClaw();
                    bot.odometry.followTrajectory(findRedBProp);

                    if (bot.pipelineRed.isPropLeft()) {
                        spikeLoc = SPIKE_LOC.LEFT;
                        if (bot.visionPortal.getCameraState() == VisionPortal.CameraState.STREAMING) {
                            bot.visionPortal.close();
                        }
                        break;
                    } else if (bot.pipelineRed.isPropCenterBack()) {
                        spikeLoc = SPIKE_LOC.CENTER;
                        if (bot.visionPortal.getCameraState() == VisionPortal.CameraState.STREAMING) {
                            bot.visionPortal.close();
                        }
                        break;
                    } else {
                        spikeLoc = SPIKE_LOC.RIGHT;
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
