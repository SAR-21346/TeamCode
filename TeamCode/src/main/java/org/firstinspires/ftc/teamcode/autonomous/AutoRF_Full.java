package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.MecanumTrain;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.vision.VisionPortal;

@Autonomous(name = "Autonomous RedFront Full")
public class AutoRF_Full extends LinearOpMode {
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

        Pose2d startPose = new Pose2d(9.5, -59, Math.toRadians(180));
        bot.odometry.setPoseEstimate(startPose);

        TrajectorySequence findRedFProp = bot.odometry.trajectorySequenceBuilder(startPose)
                .addTemporalMarker(() -> bot.closeClaw())
                .addTemporalMarker(() -> bot.target = 0)
                .lineTo(new Vector2d(11.5, -53))
                .build();

        TrajectorySequence RFdriveToRSpike = bot.odometry.trajectorySequenceBuilder(findRedFProp.end())
                .addTemporalMarker(() -> bot.closeClaw())
                .addTemporalMarker(() -> bot.target = 0)
                .splineToConstantHeading(new Vector2d(21, -48), Math.toRadians(90))
                .splineToSplineHeading(new Pose2d(23, -35, Math.toRadians(90)), Math.toRadians(90))
                .addTemporalMarker(() -> bot.openClaw())
                .waitSeconds(0.5)
                .back(4)
                .addTemporalMarker(() -> bot.closeClaw())
                .waitSeconds(0.5)
                .back(8)
                .setConstraints(bot.odometry.SHAKE_VEL_CONSTRAINT, bot.odometry.SHAKE_ACCEL_CONSTRAINT)
                .forward(4)
                .resetConstraints()
                .UNSTABLE_addDisplacementMarkerOffset(4, () -> bot.target = 140)
                .lineTo(new Vector2d(33, -43))
                .splineToSplineHeading(new Pose2d(52, -37, Math.toRadians(180)), Math.toRadians(0))
                .addTemporalMarker(() -> bot.openClaw())
                .waitSeconds(0.3)
                .addTemporalMarker(() -> {
                    bot.target = 80;
                })
                .addTemporalMarker(() -> bot.closeClaw())
                .waitSeconds(0.4)
                .UNSTABLE_addTemporalMarkerOffset(0.8, () -> bot.target = 40)
                .UNSTABLE_addTemporalMarkerOffset(1.2, () -> bot.target = 0)
                //                .lineTo(new Vector2d(36, 45)) // wall side
//                .splineToConstantHeading(new Vector2d(60, -60), Math.toRadians(0)) //wall side
                .lineTo(new Vector2d(42, 10)) // center side
                .splineToConstantHeading(new Vector2d(60, 5), Math.toRadians(0)) //center side
                .build();

        TrajectorySequence RFdriveToRSpike_2White = bot.odometry.trajectorySequenceBuilder(findRedFProp.end())
                .addTemporalMarker(() -> bot.closeClaw())
                .addTemporalMarker(() -> bot.target = 0)
                .lineTo(new Vector2d(25, -48))
                .splineToConstantHeading(new Vector2d(27.5, -30), Math.toRadians(90))
                .addTemporalMarker(() -> bot.openClaw())
                .waitSeconds(0.5)
                .back(4)
                .addTemporalMarker(() -> bot.closeClaw())
                .waitSeconds(0.5)
                .back(8)
                .setConstraints(bot.odometry.SHAKE_VEL_CONSTRAINT, bot.odometry.SHAKE_ACCEL_CONSTRAINT)
                .forward(4)
                .waitSeconds(0.5)
                .resetConstraints()
                .UNSTABLE_addDisplacementMarkerOffset(1, () -> bot.target = 140)
                .lineTo(new Vector2d(44, -36))
                .splineToConstantHeading(new Vector2d(52, -37), Math.toRadians(0))
                .addTemporalMarker(() -> bot.openClaw())
                .waitSeconds(0.3)
                .addTemporalMarker(() -> bot.target = 80)
                .addTemporalMarker(() -> bot.closeClaw())
                .waitSeconds(0.4)
                .UNSTABLE_addTemporalMarkerOffset(0.8, () -> bot.target = 40)
                .UNSTABLE_addTemporalMarkerOffset(1.2, () -> bot.target = 0)
                .lineTo(new Vector2d(34, -8))
                .addTemporalMarker(() -> bot.runIntake(0.3))
                .addTemporalMarker(() -> bot.openClaw())
                .lineTo(new Vector2d(-55, -10))
                .lineTo(new Vector2d(-63, -12))
                .waitSeconds(0.6)
                .addTemporalMarker(() -> bot.closeClaw())
                .addTemporalMarker(() -> bot.runIntake(-0.4))
                .back(4)
                .waitSeconds(0.7)
                .addTemporalMarker(() -> bot.runIntake(0))
                .lineTo(new Vector2d(24, -8))
                .addTemporalMarker(() -> bot.target = 131)
                .addTemporalMarker(() -> {/**/})
                .splineToConstantHeading(new Vector2d(52, -29), Math.toRadians(0))
                .addTemporalMarker(() -> bot.openClaw())
                .waitSeconds(0.3)
                .addTemporalMarker(() -> bot.target = 80)
                .addTemporalMarker(() -> bot.closeClaw())
                .waitSeconds(0.4)
                .UNSTABLE_addTemporalMarkerOffset(0.8, () -> bot.target = 80)
                .UNSTABLE_addTemporalMarkerOffset(1.2, () -> bot.target = 40)
                //                .lineTo(new Vector2d(36, 45)) // wall side
//                .splineToConstantHeading(new Vector2d(60, -60), Math.toRadians(0)) //wall side
                .lineTo(new Vector2d(42, 10)) // center side
                .splineToConstantHeading(new Vector2d(60, 5), Math.toRadians(0)) //center side
                .build();

        TrajectorySequence RFdriveToCSpike = bot.odometry.trajectorySequenceBuilder(findRedFProp.end())
                .addTemporalMarker(() -> bot.target = 0)
                .addTemporalMarker(() -> bot.closeClaw())
                .lineTo(new Vector2d(22, -40))
                .splineToSplineHeading(new Pose2d(19, -23, Math.toRadians(150)), Math.toRadians(150))
                .waitSeconds(0.5)
                .addTemporalMarker(() -> bot.openClaw())
                .back(4)
                .addTemporalMarker(() -> bot.closeClaw())
                .waitSeconds(0.5)
                .back(8)
                .setConstraints(bot.odometry.SHAKE_VEL_CONSTRAINT, bot.odometry.SHAKE_ACCEL_CONSTRAINT)
                .forward(4)
                .waitSeconds(0.5)
                .resetConstraints()
                .UNSTABLE_addDisplacementMarkerOffset(6, () -> bot.target = 140)
                .lineToSplineHeading(new Pose2d(52, -29, Math.toRadians(180)))
                .waitSeconds(0.5)
                .addTemporalMarker(() -> bot.openClaw())
                .waitSeconds(0.5)
                .addTemporalMarker(() -> {bot.target = 80;})
                .addTemporalMarker(() -> bot.closeClaw())
                .waitSeconds(1)
                .UNSTABLE_addTemporalMarkerOffset(0.8, () -> bot.target = 40)
                .UNSTABLE_addTemporalMarkerOffset(1.2, () -> bot.target = 0)
                //                .lineTo(new Vector2d(36, 45)) // wall side
//                .splineToConstantHeading(new Vector2d(60, -60), Math.toRadians(0)) //wall side
                .lineTo(new Vector2d(42, 10)) // center side
                .splineToConstantHeading(new Vector2d(60, 5), Math.toRadians(0)) //center side
                .build();

        TrajectorySequence RFdriveToCSpike_2White = bot.odometry.trajectorySequenceBuilder(findRedFProp.end())
                .addTemporalMarker(() -> bot.closeClaw())
                .addTemporalMarker(() -> bot.target = 0)
                .lineTo(new Vector2d(30, -36))
                .splineToConstantHeading(new Vector2d(19, -23), Math.toRadians(180))
                .waitSeconds(0.3)
                .addTemporalMarker(() -> bot.openClaw())
                .back(4)
                .addTemporalMarker(() -> bot.closeClaw())
                .waitSeconds(0.3)
                .back(8)
                .setConstraints(bot.odometry.SHAKE_VEL_CONSTRAINT, bot.odometry.SHAKE_ACCEL_CONSTRAINT)
                .forward(4)
                .waitSeconds(0.5)
                .resetConstraints()
                .UNSTABLE_addDisplacementMarkerOffset(3, () -> bot.target = 140)
                .lineToSplineHeading(new Pose2d(52, -29, Math.toRadians(180)))
                .waitSeconds(0.5)
                .addTemporalMarker(() -> bot.openClaw())
                .waitSeconds(0.5)
                .addTemporalMarker(() -> bot.target = 80)
                .addTemporalMarker(() -> bot.closeClaw())
                .UNSTABLE_addTemporalMarkerOffset(0.8, () -> bot.target = 40)
                .UNSTABLE_addTemporalMarkerOffset(1.2, () -> bot.target = 0)
                .lineTo(new Vector2d(34, -11))
                .addTemporalMarker(() -> bot.runIntake(0.3))
                .addTemporalMarker(() -> bot.openClaw())
                .lineTo(new Vector2d(-62, -10))
                .waitSeconds(0.5)
                .addTemporalMarker(() -> bot.closeClaw())
                .addTemporalMarker(() -> bot.runIntake(0))
                .lineTo(new Vector2d(24, -12))
                .addTemporalMarker(() -> bot.target = 130)
                .splineToConstantHeading(new Vector2d(52, -37), Math.toRadians(0))
                .waitSeconds(0.5)
                .addTemporalMarker(() -> bot.openClaw())
                .waitSeconds(0.5)
                .addTemporalMarker(() -> bot.target = 80)
                .addTemporalMarker(() -> bot.closeClaw())
                .UNSTABLE_addTemporalMarkerOffset(0.8, () -> bot.target = 80)
                .UNSTABLE_addTemporalMarkerOffset(1.2, () -> bot.target = 40)
                .forward(5)
                .splineToConstantHeading(new Vector2d(60, -60), Math.toRadians(0))
                .build();

        TrajectorySequence RFdriveToLSpike = bot.odometry.trajectorySequenceBuilder(findRedFProp.end())
                .addTemporalMarker(() -> bot.closeClaw())
                .addTemporalMarker(() -> bot.target = 0)
                .lineTo(new Vector2d(23, -45))
                .splineToConstantHeading(new Vector2d(4, -30), Math.toRadians(180))
                .waitSeconds(0.5)
                .addTemporalMarker(() -> bot.openClaw())
                .back(5)
                .addTemporalMarker(() -> bot.closeClaw())
                .waitSeconds(0.5)
                .back(8)
                .setConstraints(bot.odometry.SHAKE_VEL_CONSTRAINT, bot.odometry.SHAKE_ACCEL_CONSTRAINT)
                .forward(4)
                .waitSeconds(0.5)
                .resetConstraints()
                .UNSTABLE_addDisplacementMarkerOffset(4, () -> bot.target = 140)
                .lineToConstantHeading(new Vector2d(52, -24))
                .waitSeconds(.5)
                .addTemporalMarker(() -> bot.openClaw())
                .waitSeconds(.5)
                .addTemporalMarker(() -> {
                    bot.target = 80;
                })
                .addTemporalMarker(() -> bot.closeClaw())
                .waitSeconds(.5)
                .UNSTABLE_addTemporalMarkerOffset(0.8, () -> bot.target = 40)
                .UNSTABLE_addTemporalMarkerOffset(1.2, () -> bot.target = 0)
                //                .lineTo(new Vector2d(36, 45)) // wall side
//                .splineToConstantHeading(new Vector2d(60, -60), Math.toRadians(0)) //wall side
                .lineTo(new Vector2d(42, 10)) // center side
                .splineToConstantHeading(new Vector2d(60, 5), Math.toRadians(0)) //center side
                .build();

        TrajectorySequence RFdriveToLSpike_2White = bot.odometry.trajectorySequenceBuilder(findRedFProp.end())
                .addTemporalMarker(() -> bot.closeClaw())
                .addTemporalMarker(() -> bot.target = 0)
                .lineTo(new Vector2d(23, -45))
                .splineToConstantHeading(new Vector2d(4, -30), Math.toRadians(180))
                .waitSeconds(0.3)
                .addTemporalMarker(() -> bot.openClaw())
                .back(4)
                .addTemporalMarker(() -> bot.closeClaw())
                .waitSeconds(0.3)
                .back(8)
                .setConstraints(bot.odometry.SHAKE_VEL_CONSTRAINT, bot.odometry.SHAKE_ACCEL_CONSTRAINT)
                .forward(4)
                .waitSeconds(0.5)
                .resetConstraints()
                .UNSTABLE_addDisplacementMarkerOffset(2, () -> bot.target = 131)
                .setVelConstraint(bot.odometry.SLOW_VEL_CONSTRAINT)
                .lineToSplineHeading(new Pose2d(52, -22, Math.toRadians(180)))
                .resetConstraints()
                .waitSeconds(0.5)
                .addTemporalMarker(() -> bot.openClaw())
                .waitSeconds(0.5)
                .addTemporalMarker(() -> bot.target = 80)
                .addTemporalMarker(() -> bot.closeClaw())
                .UNSTABLE_addTemporalMarkerOffset(0.8, () -> bot.target = 40)
                .UNSTABLE_addTemporalMarkerOffset(1.2, () -> bot.target = 0)
                .lineTo(new Vector2d(34, -8))
                .addTemporalMarker(() -> bot.runIntake(0.3))
                .addTemporalMarker(() -> bot.openClaw())
                .lineTo(new Vector2d(-55, -10))
                .lineTo(new Vector2d(-63, -12))
                .waitSeconds(0.6)
                .addTemporalMarker(() -> bot.closeClaw())
                .addTemporalMarker(() -> bot.runIntake(-0.4))
                .back(4)
                .waitSeconds(0.7)
                .addTemporalMarker(() -> bot.runIntake(0))
                .lineTo(new Vector2d(24, -8))
                .addTemporalMarker(() -> bot.target = 131)
                .splineToConstantHeading(new Vector2d(52, -38), Math.toRadians(0))
                .waitSeconds(0.5)
                .addTemporalMarker(() -> bot.openClaw())
                .waitSeconds(0.5)
                .addTemporalMarker(() -> bot.target = 80)
                .addTemporalMarker(() -> bot.openClaw())
                .UNSTABLE_addTemporalMarkerOffset(0.8, () -> bot.target = 40)
                .UNSTABLE_addTemporalMarkerOffset(1.2, () -> bot.target = 0)
                .forward(5)
//                .lineTo(new Vector2d(36, 45)) // wall side
//                .splineToConstantHeading(new Vector2d(60, -60), Math.toRadians(0)) //wall side
                .lineTo(new Vector2d(42, 10)) // center side
                .splineToConstantHeading(new Vector2d(60, 5), Math.toRadians(0)) //center side
                .build();
        bot.arm_start = bot.outMotor.getCurrentPosition();
        bot.initEocvRed(hardwareMap);
        SPIKE_LOC spikeLoc = SPIKE_LOC.IDLE;
        bot.closeClaw();
        waitForStart();
        runtime.reset();
        double x = 0;

        if (opModeIsActive()) {
            while (opModeIsActive()) {
                while (runtime.seconds() < 2) {
                    bot.closeClaw();
                    bot.odometry.followTrajectorySequence(findRedFProp);

                    if (bot.pipelineRed.isPropRight()) {
                        spikeLoc = SPIKE_LOC.RIGHT;
                        telemetry.addData("x", bot.pipelineRed.centerX);
                        if (bot.visionPortal.getCameraState() == VisionPortal.CameraState.STREAMING) {
                            bot.visionPortal.close();
                        }
                        break;
                    } else if (bot.pipelineRed.isPropCenter()) {
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
                            bot.odometry.followTrajectorySequenceAsync(RFdriveToRSpike);
//                            bot.odometry.followTrajectorySequenceAsync(RFdriveToRSpike_2White);
                            spikeLoc = SPIKE_LOC.IDLE;
                            break;
                        }
                    case CENTER:
                        telemetry.addLine("CENTER SPIKE");
                        if (!bot.odometry.isBusy()) {
                            bot.odometry.followTrajectorySequenceAsync(RFdriveToCSpike);
//                            bot.odometry.followTrajectorySequenceAsync(RFdriveToCSpike_2White);
                            spikeLoc = SPIKE_LOC.IDLE;
                            break;
                        }
                    case LEFT:
                        telemetry.addLine("LEFT SPIKE");
                        if (!bot.odometry.isBusy()) {
                            bot.odometry.followTrajectorySequenceAsync(RFdriveToLSpike);
//                            bot.odometry.followTrajectorySequenceAsync(RFdriveToLSpike_2White);
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