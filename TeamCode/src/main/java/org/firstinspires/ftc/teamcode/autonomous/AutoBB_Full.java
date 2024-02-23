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

@Autonomous(name = "Autonomous BlueBack Full")
public class AutoBB_Full extends LinearOpMode {
    MecanumTrain bot;

    enum SPIKE_LOC  {
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

        Pose2d startPose = new Pose2d(-34, 59, Math.toRadians(0));
        bot.odometry.setPoseEstimate(startPose);

        TrajectorySequence findBlueBProp = bot.odometry.trajectorySequenceBuilder(startPose)
                .addTemporalMarker(() -> bot.closeClaw())
                .lineTo(new Vector2d(-40, 53))
                .build();

        TrajectorySequence BBdriveToLSpike = bot.odometry.trajectorySequenceBuilder(findBlueBProp.end())
                .addTemporalMarker(() -> bot.closeClaw())
                .addTemporalMarker(() -> bot.target = 0)
                .setVelConstraint(bot.odometry.SLOW_VEL_CONSTRAINT)
                .lineTo(new Vector2d(-40, 55))
                .waitSeconds(1)
                .lineToSplineHeading(new Pose2d(-39, 37, Math.toRadians(0)))
                .splineToConstantHeading(new Vector2d(-28, 32), Math.toRadians(0))
                .addTemporalMarker(() -> bot.openClaw())
                .waitSeconds(0.5)
                .back(5)
                .addTemporalMarker(() -> bot.closeClaw())
                .waitSeconds(0.5)
                .back(8)
                .lineTo(new Vector2d(-40, 55))
                .UNSTABLE_addDisplacementMarkerOffset(81, () -> bot.target = 135)
                .lineTo(new Vector2d(14, 55))
                .waitSeconds(0.5)
                .setAccelConstraint(bot.odometry.SHAKE_ACCEL_CONSTRAINT)
                .forward(4)
                .waitSeconds(0.5)
                .resetAccelConstraint()
                .lineTo(new Vector2d(18, 46))
                .splineToSplineHeading(new Pose2d(52, 39, Math.toRadians(180)), Math.toRadians(0))
                .waitSeconds(0.5)
                .addTemporalMarker(() -> bot.openClaw())
                .waitSeconds(0.5)
                .addTemporalMarker(() -> bot.target = 80)
                .addTemporalMarker(() -> bot.closeClaw())
                .waitSeconds(1)
                .UNSTABLE_addTemporalMarkerOffset(0.8, () -> bot.target = 40)
                .UNSTABLE_addTemporalMarkerOffset(1.2, () -> bot.target = 0)
                .lineTo(new Vector2d(36, 45))
                .splineToConstantHeading(new Vector2d(60, 60), Math.toRadians(0))
                .build();

        TrajectorySequence BBdriveToCSpike = bot.odometry.trajectorySequenceBuilder(findBlueBProp.end())
                .addTemporalMarker(() -> bot.closeClaw())
                .addTemporalMarker(() -> bot.target = 0)
                .setVelConstraint(bot.odometry.SLOW_VEL_CONSTRAINT)
                .lineTo(new Vector2d(-40, 50))
                .waitSeconds(1)
                .lineTo(new Vector2d(-43, 44))
                .splineToSplineHeading(new Pose2d(-41, 24, Math.toRadians(315)), Math.toRadians(315))
                .waitSeconds(0.5)
                .addTemporalMarker(() -> bot.openClaw())
                .back(4)
                .addTemporalMarker(() -> bot.closeClaw())
                .waitSeconds(0.5)
                .back(4)
                .splineToSplineHeading(new Pose2d(-40, 55, Math.toRadians(0)), Math.toRadians(0))
                .UNSTABLE_addDisplacementMarkerOffset(93, () -> bot.target = 138)
                .lineTo(new Vector2d(14, 55))
                .setAccelConstraint(bot.odometry.SHAKE_ACCEL_CONSTRAINT)
                .forward(4)
                .back(6)
                .forward(6)
                .back(2)
                .lineTo(new Vector2d(14, 44))
                .splineToSplineHeading(new Pose2d(52, 29, Math.toRadians(180)), Math.toRadians(0))
                .waitSeconds(0.5)
                .addTemporalMarker(() -> bot.openClaw())
                .waitSeconds(0.5)
                .addTemporalMarker(() -> bot.target = 80)
                .addTemporalMarker(() -> bot.closeClaw())
                .waitSeconds(1)
                .UNSTABLE_addTemporalMarkerOffset(0.8, () -> bot.target = 40)
                .UNSTABLE_addTemporalMarkerOffset(1.2, () -> bot.target = 0)
                .lineTo(new Vector2d(36, 45))
                .splineToConstantHeading(new Vector2d(60, 60), Math.toRadians(0))
                .build();

        TrajectorySequence BBdriveToRSpike = bot.odometry.trajectorySequenceBuilder(findBlueBProp.end())
                .addTemporalMarker(() -> bot.closeClaw())
                .addTemporalMarker(() -> bot.target = 0)
                .setVelConstraint(bot.odometry.SLOW_VEL_CONSTRAINT)
                .waitSeconds(0.5)
                .back(4)
                .splineToSplineHeading(new Pose2d(-48, 33, Math.toRadians(270)), Math.toRadians(270))
                .waitSeconds(0.5)
                .addTemporalMarker(() -> bot.openClaw())
                .back(4)
                .addTemporalMarker(() -> bot.closeClaw())
                .waitSeconds(0.5)
                .back(4)
                .lineTo(new Vector2d(-40, 55))
                .UNSTABLE_addDisplacementMarkerOffset(67, () -> bot.target = 138)
                .turn(Math.toRadians(90))
                .lineTo(new Vector2d(14, 55))
                .waitSeconds(0.5)
                .setAccelConstraint(bot.odometry.SHAKE_ACCEL_CONSTRAINT)
                .forward(6)
                .waitSeconds(0.5)
                .resetAccelConstraint()
                .splineToSplineHeading(new Pose2d(48, 24, Math.toRadians(180)), Math.toRadians(0))
                .back(4)
                .waitSeconds(0.5)
                .addTemporalMarker(() -> bot.openClaw())
                .waitSeconds(0.5)
                .addTemporalMarker(() -> bot.target = 80)
                .addTemporalMarker(() -> bot.closeClaw())
                .waitSeconds(1)
                .UNSTABLE_addTemporalMarkerOffset(0.8, () -> bot.target = 40)
                .UNSTABLE_addTemporalMarkerOffset(1.2, () -> bot.target = 0)
                .lineTo(new Vector2d(36,45))
                .splineToConstantHeading(new Vector2d(60, 60), Math.toRadians(0))
                .build();

        bot.initEocvBlue(hardwareMap);
        SPIKE_LOC spikeLoc = SPIKE_LOC.IDLE;
        bot.closeClaw();
        waitForStart();
        runtime.reset();
        double x = 0;

        if (opModeIsActive()) {
            while (opModeIsActive()) {
                while (runtime.seconds() < 1) {
                    bot.closeClaw();
                    bot.odometry.followTrajectorySequence(findBlueBProp);

                    if (bot.pipeline.isPropRight()){
                        spikeLoc = SPIKE_LOC.RIGHT;
                        if (bot.visionPortal.getCameraState() == VisionPortal.CameraState.STREAMING) {
                            bot.visionPortal.close();
                        }
                        break;
                    } else if (bot.pipeline.isPropCenterBack()){
                        spikeLoc = SPIKE_LOC.CENTER;
                        if (bot.visionPortal.getCameraState() == VisionPortal.CameraState.STREAMING) {
                            bot.visionPortal.close();
                        }
                        break;
                    } else {
                        spikeLoc = SPIKE_LOC.LEFT;
                        if (bot.visionPortal.getCameraState() == VisionPortal.CameraState.STREAMING) {
                            bot.visionPortal.close();
                        }
                    }
                    telemetry.addData("x", bot.pipeline.centerX);
                    sleep(20);
                }


                if (bot.visionPortal.getCameraState() == VisionPortal.CameraState.CAMERA_DEVICE_READY) {
                    bot.visionPortal.close();
                }


                switch (spikeLoc) {
                    case RIGHT:
                        telemetry.addLine("RIGHT SPIKE");
                        if (!bot.odometry.isBusy()) {
                            bot.odometry.followTrajectorySequenceAsync(BBdriveToRSpike);
                            spikeLoc = SPIKE_LOC.IDLE;
                            break;
                        }
                    case CENTER:
                        telemetry.addLine("CENTER SPIKE");
                        if (!bot.odometry.isBusy()) {
                            bot.odometry.followTrajectorySequenceAsync(BBdriveToCSpike);
                            spikeLoc = SPIKE_LOC.IDLE;
                            break;
                        }
                    case LEFT:
                        telemetry.addLine("LEFT SPIKE");
                        if (!bot.odometry.isBusy()) {
                            bot.odometry.followTrajectorySequenceAsync(BBdriveToLSpike);
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
