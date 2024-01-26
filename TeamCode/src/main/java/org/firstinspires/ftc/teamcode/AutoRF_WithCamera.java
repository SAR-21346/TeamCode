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
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.TrajectoryLibrary;
import org.firstinspires.ftc.vision.VisionPortal;

import java.util.List;

@Autonomous(name = "Autonomous RedFront With Camera")
public class AutoRF_WithCamera extends LinearOpMode {
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

        Pose2d startPose = new Pose2d(10.5, -59, Math.toRadians(180));
        bot.odometry.setPoseEstimate(startPose);

        Trajectory findRedFProp = bot.odometry.trajectoryBuilder(startPose)
                .lineTo(new Vector2d(12, -53))
                .build();

        TrajectorySequence RFdriveToRSpike = bot.odometry.trajectorySequenceBuilder(findRedFProp.end())
                .splineToSplineHeading(new Pose2d(23, -38, Math.toRadians(90)), Math.toRadians(0))
                .lineTo(new Vector2d(23, -35))
                .addTemporalMarker(() -> bot.openClaw())
                .waitSeconds(0.5)
                .back(4)
                .waitSeconds(0.5)
                .addTemporalMarker(() -> bot.closeClaw())
                .waitSeconds(0.5)
                .lineTo(new Vector2d(28, -39))
                .UNSTABLE_addDisplacementMarkerOffset(8, () -> bot.target = 130)
                .splineToSplineHeading(new Pose2d(52, -39, Math.toRadians(180)), Math.toRadians(0))
                .setAccelConstraint(bot.odometry.SHAKE_ACCEL_CONSTRAINT)
                .forward(4)
                .back(4)
                .resetAccelConstraint()
                .waitSeconds(0.5)
                .addTemporalMarker(() -> bot.openClaw())
                .waitSeconds(0.5)
                .addTemporalMarker(() -> {bot.target = 80;})
                .addTemporalMarker(() -> bot.closeClaw())
                .waitSeconds(1)
                .lineTo(new Vector2d(45, -18))
                .addTemporalMarker(() -> {bot.target = 40;})
                .waitSeconds(0.4)
                .addTemporalMarker(() -> {bot.target = 0;})
                .lineTo(new Vector2d(58, -5))
                .build();

        TrajectorySequence RFdriveToCSpike = bot.odometry.trajectorySequenceBuilder(findRedFProp.end())
                .lineTo(new Vector2d(14, -40))
                .splineToSplineHeading(new Pose2d(19, -23, Math.toRadians(150)), Math.toRadians(90))
                .waitSeconds(0.5)
                .addTemporalMarker(() -> bot.openClaw())
                .back(5)
                .addTemporalMarker(() -> bot.closeClaw())
                .waitSeconds(0.5)
                .UNSTABLE_addDisplacementMarkerOffset(8, () -> {bot.target = 130;})
                .lineToSplineHeading(new Pose2d(52, -29, Math.toRadians(180)))
                .setAccelConstraint(SampleMecanumDrive.SHAKE_ACCEL_CONSTRAINT)
                .forward(6)
                .back(6)
                .resetAccelConstraint()
                .waitSeconds(0.5)
                .addTemporalMarker(() -> bot.openClaw())
                .waitSeconds(0.8)
                .addTemporalMarker(() -> {bot.target = 80;})
                .addTemporalMarker(() -> bot.closeClaw())
                .waitSeconds(1)
                .lineTo(new Vector2d(34, -18))
                .addTemporalMarker(() -> {bot.target = 40;})
                .waitSeconds(0.4)
                .addTemporalMarker(() -> {bot.target = 0;})
                .splineToConstantHeading(new Vector2d(60, -5), Math.toRadians(0))
                .build();

        TrajectorySequence RFdriveToLSpike = bot.odometry.trajectorySequenceBuilder(findRedFProp.end())
                .lineTo(new Vector2d(14, -40))
                .splineToConstantHeading(new Vector2d(4, -29), Math.toRadians(180))
                .waitSeconds(0.5)
                .addTemporalMarker(() -> bot.openClaw())
                .back(6)
                .addTemporalMarker(() -> bot.closeClaw())
                .waitSeconds(0.5)
                .addTemporalMarker(5.5, () -> {bot.target = 130;})
                .lineToConstantHeading(new Vector2d(52, -24))
                .setAccelConstraint(bot.odometry.SHAKE_ACCEL_CONSTRAINT)
                .forward(5)
                .back(5)
                .resetAccelConstraint()
                .waitSeconds(0.5)
                .addTemporalMarker(() -> bot.openClaw())
                .waitSeconds(0.8)
                .addTemporalMarker(() -> {bot.target = 80;})
                .addTemporalMarker(() -> bot.closeClaw())
                .waitSeconds(1)
                .lineTo(new Vector2d(34, -18))
                .addTemporalMarker(() -> {bot.target = 40;})
                .waitSeconds(0.4)
                .addTemporalMarker(() -> {bot.target = 0;})
                .splineToConstantHeading(new Vector2d(60, -5), Math.toRadians(0))
                .build();

        bot.initTfodRed(hardwareMap);
        SPIKE_LOC spikeLoc = SPIKE_LOC.IDLE;
        waitForStart();
        bot.closeClaw();
        runtime.reset();
        double x = 0;

        if (opModeIsActive()) {
            while (opModeIsActive()) {
                while (runtime.seconds() < 3) {
                    bot.closeClaw();
                    bot.odometry.followTrajectory(findRedFProp);

                    List<Recognition> currentRecognitions = bot.tfod.getRecognitions();
                    if (bot.tfod.getRecognitions() != null) {
                        for (Recognition recognition : currentRecognitions) {
                            x = (recognition.getLeft() + recognition.getRight()) / 2;
                            telemetry.addData("x", x);
                        }
                        // Changed 1/13/2024 @ 2:43 PM
                        if (x > 550) {
                            spikeLoc = SPIKE_LOC.RIGHT;
                            if (bot.visionPortal.getCameraState() == VisionPortal.CameraState.STREAMING) {
                                bot.visionPortal.close();
                            }
                            break;
                        } else if (x < 550 && x > 200) {
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
                            break;
                        }

                    }
                }

                if (bot.visionPortal.getCameraState() == VisionPortal.CameraState.STREAMING) {
                    bot.visionPortal.close();
                }

                switch (spikeLoc) {
                    case RIGHT:
                        telemetry.addLine("RIGHT SPIKE");
                        if (!bot.odometry.isBusy()) {
                            bot.odometry.followTrajectorySequenceAsync(RFdriveToRSpike);
                            spikeLoc = SPIKE_LOC.IDLE;
                            break;
                        }
                    case CENTER:
                        telemetry.addLine("CENTER SPIKE");
                        if (!bot.odometry.isBusy()) {
                            bot.odometry.followTrajectorySequenceAsync(RFdriveToCSpike);
                            spikeLoc = SPIKE_LOC.IDLE;
                            break;
                        }
                    default:
                        telemetry.addLine("LEFT SPIKE");
                        if (!bot.odometry.isBusy()) {
                            bot.odometry.followTrajectorySequenceAsync(RFdriveToLSpike);
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
