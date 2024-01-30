package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.STOP_AND_RESET_ENCODER;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.vision.VisionPortal;

@Autonomous(name = "Autonomous RedFront With Camera")
public class AutoRF_WithCamera extends LinearOpMode {
    MecanumTrain bot;

    enum SPIKE_LOC {
        LEFT,
        CENTER,
        RIGHT,
        IDLE
    }

    @Override
    public void runOpMode() {
        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        ElapsedTime runtime = new ElapsedTime();
        bot = new MecanumTrain(hardwareMap, runtime);

        Pose2d startPose = new Pose2d(10.5, -60, Math.toRadians(180));
        bot.odometry.setPoseEstimate(startPose);

        bot.outMotor.setMode(STOP_AND_RESET_ENCODER);
        Trajectory findRedFProp = bot.odometry.trajectoryBuilder(startPose)
                .lineTo(new Vector2d(12, -53))
                .build();

        TrajectorySequence RFdriveToRSpike = bot.odometry.trajectorySequenceBuilder(findRedFProp.end())
                .addTemporalMarker(() -> bot.target = 0)
                .splineToSplineHeading(new Pose2d(23, -38, Math.toRadians(90)), Math.toRadians(0))
                .forward(3)
                .addTemporalMarker(() -> bot.openClaw())
                .waitSeconds(0.5)
                .back(4)
                .waitSeconds(0.5)
                .addTemporalMarker(() -> bot.closeClaw())
                .waitSeconds(0.5)
                .setAccelConstraint(bot.odometry.SHAKE_ACCEL_CONSTRAINT)
                .forward(4)
                .back(4)
                .resetAccelConstraint()
                .lineTo(new Vector2d(28, -39))
                .splineToSplineHeading(new Pose2d(52, -39, Math.toRadians(180)), Math.toRadians(0))
                .addTemporalMarker(() -> bot.target = 40)
                .waitSeconds(0.2)
                .addTemporalMarker(() -> bot.target = 80)
                .waitSeconds(0.2)
                .addTemporalMarker(() -> bot.target = 120)
                .waitSeconds(0.7)
                .addTemporalMarker(() -> bot.openClaw())
                .waitSeconds(0.5)
                .addTemporalMarker(() -> {
                    bot.target = 80;
                })
                .addTemporalMarker(() -> bot.closeClaw())
                .waitSeconds(1)
                .lineTo(new Vector2d(45, -18))
                .addTemporalMarker(() -> {
                    bot.target = 40;
                })
                .waitSeconds(0.4)
                .addTemporalMarker(() -> {
                    bot.target = 0;
                })
                .lineTo(new Vector2d(58, -5))
                .build();

        TrajectorySequence RFdriveToCSpike = bot.odometry.trajectorySequenceBuilder(findRedFProp.end())
                .addTemporalMarker(() -> bot.target = 0)
                .lineTo(new Vector2d(14, -40))
                .splineToSplineHeading(new Pose2d(19, -23, Math.toRadians(150)), Math.toRadians(90))
                .waitSeconds(0.5)
                .addTemporalMarker(() -> bot.openClaw())
                .back(5)
                .addTemporalMarker(() -> bot.closeClaw())
                .waitSeconds(0.5)
                .setAccelConstraint(bot.odometry.SHAKE_ACCEL_CONSTRAINT)
                .forward(6)
                .back(6)
                .resetAccelConstraint()
                .lineToSplineHeading(new Pose2d(52, -29, Math.toRadians(180)))
                .addTemporalMarker(() -> bot.target = 40)
                .waitSeconds(0.2)
                .addTemporalMarker(() -> bot.target = 80)
                .waitSeconds(0.2)
                .addTemporalMarker(() -> bot.target = 120)
                .waitSeconds(0.7)
                .addTemporalMarker(() -> bot.openClaw())
                .waitSeconds(0.8)
                .addTemporalMarker(() -> {
                    bot.target = 80;
                })
                .addTemporalMarker(() -> bot.closeClaw())
                .waitSeconds(1)
                .lineTo(new Vector2d(34, -18))
                .addTemporalMarker(() -> {
                    bot.target = 40;
                })
                .waitSeconds(0.4)
                .addTemporalMarker(() -> {
                    bot.target = 0;
                })
                .splineToConstantHeading(new Vector2d(60, -5), Math.toRadians(0))
                .build();

        TrajectorySequence RFdriveToLSpike = bot.odometry.trajectorySequenceBuilder(findRedFProp.end())
                .addTemporalMarker(() -> bot.target = 0)
                .splineToConstantHeading(new Vector2d(4, -29), Math.toRadians(180))
                .waitSeconds(0.5)
                .addTemporalMarker(() -> bot.openClaw())
                .back(6)
                .addTemporalMarker(() -> bot.closeClaw())
                .waitSeconds(0.5)
                .setAccelConstraint(bot.odometry.SHAKE_ACCEL_CONSTRAINT)
                .forward(5)
                .back(5)
                .resetAccelConstraint()
                .lineToConstantHeading(new Vector2d(52, -24))
                .addTemporalMarker(() -> bot.target = 40)
                .waitSeconds(0.2)
                .addTemporalMarker(() -> bot.target = 80)
                .waitSeconds(0.2)
                .addTemporalMarker(() -> bot.target = 120)
                .waitSeconds(0.7)
                .addTemporalMarker(() -> bot.openClaw())
                .waitSeconds(0.8)
                .addTemporalMarker(() -> {
                    bot.target = 80;
                })
                .addTemporalMarker(() -> bot.closeClaw())
                .waitSeconds(1)
                .lineTo(new Vector2d(34, -18))
                .addTemporalMarker(() -> {
                    bot.target = 40;
                })
                .waitSeconds(0.4)
                .addTemporalMarker(() -> {
                    bot.target = 0;
                })
                .splineToConstantHeading(new Vector2d(60, -10), Math.toRadians(0))
                .build();


        bot.initEocvRed(hardwareMap);
        SPIKE_LOC spikeLoc = SPIKE_LOC.IDLE;
        waitForStart();
        bot.closeClaw();
        runtime.reset();
        double x = 0;

        if (opModeIsActive()) {
            while (opModeIsActive()) {
                while (runtime.seconds() < 2) {
                    bot.closeClaw();
                    bot.odometry.followTrajectory(findRedFProp);

                    if (bot.pipelineRed.isPropRight()) {
                        spikeLoc = SPIKE_LOC.RIGHT;
                        if (bot.visionPortal.getCameraState() == VisionPortal.CameraState.STREAMING) {
                            bot.visionPortal.close();
                        }
                        break;
                    } else if (bot.pipelineRed.isPropCenter()) {
                        spikeLoc = SPIKE_LOC.CENTER;
                        telemetry.addLine("CENTER SPIKE");
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
                    case LEFT:
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