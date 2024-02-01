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

        Pose2d startPose = new Pose2d(-35, 59, Math.toRadians(0));
        bot.odometry.setPoseEstimate(startPose);

        Trajectory findBlueBProp = bot.odometry.trajectoryBuilder(startPose)
                .lineTo(new Vector2d(-35, 53))
                .build();

        // TODO: Blue-Back Drive Trajectories
        TrajectorySequence BBdriveToLSpike = bot.odometry.trajectorySequenceBuilder(findBlueBProp.end())
                .splineToSplineHeading(new Pose2d(-46, 35, Math.toRadians(270)), Math.toRadians(270))
                .waitSeconds(0.5)
                .addTemporalMarker(() -> bot.openClaw())
                .back(6)
                .addTemporalMarker(() -> bot.closeClaw())
                .waitSeconds(0.5)
                .strafeRight(3)
                .splineToSplineHeading(new Pose2d(-47, 9.5, Math.toRadians(90)), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(5, 13), Math.toRadians(0))
                .UNSTABLE_addDisplacementMarkerOffset(24, () -> bot.target = 130)
                .splineToSplineHeading(new Pose2d(52, 33, Math.toRadians(180)), Math.toRadians(0))
                .setAccelConstraint(bot.odometry.SHAKE_ACCEL_CONSTRAINT)
                .forward(6)
                .back(6)
                .resetAccelConstraint()
                .waitSeconds(0.5)
                .addTemporalMarker(() -> bot.openClaw())
                .waitSeconds(0.5)
                .addTemporalMarker(() -> bot.target = 80)
                .addTemporalMarker(() -> bot.closeClaw())
                .waitSeconds(1)
                .lineTo(new Vector2d(36, 45))
                .addTemporalMarker(() -> bot.target = 40)
                .waitSeconds(0.4)
                .addTemporalMarker(() -> bot.target = 0)
                .splineToConstantHeading(new Vector2d(60, 60), Math.toRadians(0))
                .build();

        TrajectorySequence BBdriveToCSpike = bot.odometry.trajectorySequenceBuilder(findBlueBProp.end())
                .splineToSplineHeading(new Pose2d(-41, 24, Math.toRadians(315)), Math.toRadians(315))
                .waitSeconds(0.5)
                .addTemporalMarker(() -> bot.openClaw())
                .back(6)
                .addTemporalMarker(() -> bot.closeClaw())
                .waitSeconds(0.5)
                .strafeRight(3)
                .splineToSplineHeading(new Pose2d(-47, 9.5, Math.toRadians(90)), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(5, 13), Math.toRadians(0))
                .UNSTABLE_addDisplacementMarkerOffset(24, () -> bot.target = 130)
                .splineToSplineHeading(new Pose2d(52, 33, Math.toRadians(180)), Math.toRadians(0))
                .setAccelConstraint(bot.odometry.SHAKE_ACCEL_CONSTRAINT)
                .forward(6)
                .back(6)
                .resetAccelConstraint()
                .waitSeconds(0.5)
                .addTemporalMarker(() -> bot.openClaw())
                .waitSeconds(0.5)
                .addTemporalMarker(() -> bot.target = 80)
                .addTemporalMarker(() -> bot.closeClaw())
                .waitSeconds(1)
                .lineTo(new Vector2d(36, 45))
                .addTemporalMarker(() -> bot.target = 40)
                .waitSeconds(0.4)
                .addTemporalMarker(() -> bot.target = 0)
                .splineToConstantHeading(new Vector2d(60, 60), Math.toRadians(0))
                .build();

        TrajectorySequence BBdriveToRSpike = bot.odometry.trajectorySequenceBuilder(findBlueBProp.end())
                .lineToSplineHeading(new Pose2d(-38, 40, Math.toRadians(0)))
                .splineToConstantHeading(new Vector2d(-28, 34), Math.toRadians(180))
                .addTemporalMarker(() -> bot.openClaw())
                .waitSeconds(0.5)
                .back(5)
                .waitSeconds(0.5)
                .addTemporalMarker(() -> bot.closeClaw())
                .waitSeconds(0.5)
                .strafeRight(5)
                .splineToConstantHeading(new Vector2d(-27, 11), Math.toRadians(0))
                .UNSTABLE_addDisplacementMarkerOffset(48, () -> bot.target = 130)
                .lineToConstantHeading(new Vector2d(10, 11))
                .splineToSplineHeading(new Pose2d(52, 39, Math.toRadians(180)), Math.toRadians(0))
                .setAccelConstraint(bot.odometry.SHAKE_ACCEL_CONSTRAINT)
                .forward(6)
                .back(6)
                .resetAccelConstraint()
                .waitSeconds(0.5)
                .addTemporalMarker(() -> bot.openClaw())
                .waitSeconds(0.5)
                .addTemporalMarker(() -> bot.target = 80)
                .addTemporalMarker(() -> bot.closeClaw())
                .waitSeconds(1)
                .lineTo(new Vector2d(36,45))
                .addTemporalMarker(() -> bot.target = 40)
                .waitSeconds(0.4)
                .addTemporalMarker(() -> bot.target = 0)
                .splineToConstantHeading(new Vector2d(60, 60), Math.toRadians(0))
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
                    bot.odometry.followTrajectory(findBlueBProp);

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
