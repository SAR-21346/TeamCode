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

        Pose2d startPose = new Pose2d(10.5, 57, Math.toRadians(0));
        bot.odometry.setPoseEstimate(startPose);

        TrajectorySequence findBlueFProp = bot.odometry.trajectorySequenceBuilder(startPose)
                .addTemporalMarker(() -> bot.closeClaw())
                .lineTo(new Vector2d(18, 53))
                .build();

        TrajectorySequence BFdriveToLSpike = bot.odometry.trajectorySequenceBuilder(findBlueFProp.end())
                .addTemporalMarker(() -> bot.closeClaw())
                .addTemporalMarker(() -> bot.target = 0)
                .setVelConstraint(bot.odometry.SLOW_VEL_CONSTRAINT)
                .splineToConstantHeading(new Vector2d(23, 48), Math.toRadians(270))
                .splineToSplineHeading(new Pose2d(23, 35, Math.toRadians(270)), Math.toRadians(270))
                .waitSeconds(0.5)
                .addTemporalMarker(() -> bot.openClaw())
                .back(4)
                .addTemporalMarker(() -> bot.closeClaw())
                .waitSeconds(0.5)
                .back(8)
                .waitSeconds(0.5)
                .UNSTABLE_addDisplacementMarkerOffset(4, () -> {bot.target = 138;})
                .back(6)
                .setConstraints(bot.odometry.SHAKE_VEL_CONSTRAINT, bot.odometry.SHAKE_ACCEL_CONSTRAINT)
                .forward(4)
                .back(6)
                .resetConstraints()
                .lineTo(new Vector2d(33, 43))
                .splineToSplineHeading(new Pose2d(52, 37, Math.toRadians(180)), Math.toRadians(0))
                .addTemporalMarker(() -> bot.openClaw())
                .waitSeconds(0.5)
                .addTemporalMarker(() -> {bot.target = 80;})
                .addTemporalMarker(() -> bot.closeClaw())
                .UNSTABLE_addTemporalMarkerOffset(0.8, () -> bot.target = 40)
                .UNSTABLE_addTemporalMarkerOffset(1.2, () -> bot.target = 0)
                .lineTo(new Vector2d(36, 45)) // wall side
                .splineToConstantHeading(new Vector2d(60, 60), Math.toRadians(0)) //wall side
//                .lineTo(new Vector2d(42, 10)) // center side
//                .splineToConstantHeading(new Vector2d(60, 5), Math.toRadians(0)) //center side
                .build();
        TrajectorySequence BFdriveToLSpike_2White = bot.odometry.trajectorySequenceBuilder(findBlueFProp.end())
                .addTemporalMarker(() -> bot.closeClaw()) // close claw
                .addTemporalMarker(() -> bot.target = 0) // target = 0
                .lineTo(new Vector2d(14, 40))
                .splineToConstantHeading(new Vector2d(23, 35), Math.toRadians(0))
                .waitSeconds(0.5)
                .addTemporalMarker(() -> bot.openClaw()) // open claw
                .back(4)
                .addTemporalMarker(() -> bot.closeClaw()) // close claw
                .waitSeconds(0.5)
                .UNSTABLE_addDisplacementMarkerOffset(15, () -> bot.target = 140)
                .back(6)
                .setConstraints(bot.odometry.SHAKE_VEL_CONSTRAINT, bot.odometry.SHAKE_ACCEL_CONSTRAINT)
                .forward(4)
                .waitSeconds(0.5)
                .resetConstraints()
                .lineTo(new Vector2d(24, 52))
                .splineToSplineHeading(new Pose2d(52, 37, Math.toRadians(180)), Math.toRadians(0))
                .addTemporalMarker(() -> bot.openClaw()) // open claw
                .waitSeconds(0.5)
                .addTemporalMarker(() -> bot.target = 80) // target = 80
                .addTemporalMarker(() -> bot.closeClaw()) // close claw
                .UNSTABLE_addTemporalMarkerOffset(0.8, () -> bot.target = 40) // target = 40
                .UNSTABLE_addTemporalMarkerOffset(1.2, () -> bot.target = 0) // target = 0
                .lineTo(new Vector2d(34, 18))
                .addTemporalMarker(() -> bot.openClaw()) // open claw
                .addTemporalMarker(() -> bot.runIntake(0.3)) // intake on
                .lineTo(new Vector2d(-59, 8))
                .addTemporalMarker(() -> bot.runIntake(0)) // intake off
                .addTemporalMarker(() -> bot.closeClaw()) // close claw
                .lineTo(new Vector2d(24, 12))
                .addTemporalMarker(() -> bot.target = 140) // arm up
                .splineToConstantHeading(new Vector2d(52, 37), Math.toRadians(0))
                .waitSeconds(0.5)
                .addTemporalMarker(() -> bot.openClaw()) // open claw
                .waitSeconds(0.5)
                .addTemporalMarker(() -> bot.target = 80) // target = 80
                .addTemporalMarker(() -> bot.closeClaw()) // close claw
                .waitSeconds(0.5)
                .UNSTABLE_addTemporalMarkerOffset(0.8, () -> bot.target = 40) // target = 40
                .UNSTABLE_addTemporalMarkerOffset(1.2, () -> bot.target = 0) // target = 0
                .lineTo(new Vector2d(36, 45)) // wall side
                .splineToConstantHeading(new Vector2d(60, 60), Math.toRadians(0)) //wall side
//                .lineTo(new Vector2d(42, 10)) // center side
//                .splineToConstantHeading(new Vector2d(60, 5), Math.toRadians(0)) //center side
                .build();

        TrajectorySequence BFdriveToCSpike = bot.odometry.trajectorySequenceBuilder(findBlueFProp.end())
                .addTemporalMarker(() -> bot.target = 0)
                .addTemporalMarker(() -> bot.closeClaw())
                .setVelConstraint(bot.odometry.SLOW_VEL_CONSTRAINT)
                .splineToSplineHeading(new Pose2d(19, 23, Math.toRadians(230)), Math.toRadians(230))
                .waitSeconds(0.5)
                .addTemporalMarker(() -> bot.openClaw())
                .back(4)
                .addTemporalMarker(() -> bot.closeClaw())
                .waitSeconds(0.5)
                .back(8)
                .setConstraints(bot.odometry.SHAKE_VEL_CONSTRAINT, bot.odometry.SHAKE_ACCEL_CONSTRAINT)
                .forward(4)
                .back(6)
                .resetConstraints()
                .UNSTABLE_addDisplacementMarkerOffset(6, () -> {bot.target = 140;})
                .lineToSplineHeading(new Pose2d(52, 29, Math.toRadians(180)))
                .waitSeconds(0.5)
                .addTemporalMarker(() -> bot.openClaw())
                .waitSeconds(0.5)
                .addTemporalMarker(() -> {bot.target = 80;})
                .addTemporalMarker(() -> bot.closeClaw())
                .waitSeconds(0.5)
                .UNSTABLE_addTemporalMarkerOffset(0.8, () -> bot.target = 40)
                .UNSTABLE_addTemporalMarkerOffset(1.2, () -> bot.target = 0)
                .lineTo(new Vector2d(36, 45)) // wall side
                .splineToConstantHeading(new Vector2d(60, 60), Math.toRadians(0)) //wall side
//                .lineTo(new Vector2d(42, 10)) // center side
//                .splineToConstantHeading(new Vector2d(60, 5), Math.toRadians(0)) //center side
                .build();

        TrajectorySequence BFdriveToRSpike = bot.odometry.trajectorySequenceBuilder(findBlueFProp.end())
                .addTemporalMarker(() -> bot.closeClaw())
                .addTemporalMarker(() -> bot.target = 0)
                .setVelConstraint(bot.odometry.SLOW_VEL_CONSTRAINT)
                .lineTo(new Vector2d(23, 45))
                .turn(Math.toRadians(180))
                .splineToConstantHeading(new Vector2d(4, 30), Math.toRadians(180))
                .waitSeconds(.5)
                .addTemporalMarker(() -> bot.openClaw())
                .back(5)
                .addTemporalMarker(() -> bot.closeClaw())
                .waitSeconds(.5)
                .back(8)
                .setConstraints(bot.odometry.SHAKE_VEL_CONSTRAINT, bot.odometry.SHAKE_ACCEL_CONSTRAINT)
                .forward(4)
                .back(6)
                .resetConstraints()
                .UNSTABLE_addDisplacementMarkerOffset(6, () -> {
                    bot.target = 140;
                })
                .lineToConstantHeading(new Vector2d(52, 24))
                .waitSeconds(.5)
                .addTemporalMarker(() -> bot.openClaw())
                .waitSeconds(.5)
                .addTemporalMarker(() -> {bot.target = 80;})
                .addTemporalMarker(() -> bot.closeClaw())
                .waitSeconds(.5)
                .UNSTABLE_addTemporalMarkerOffset(0.8, () -> bot.target = 40)
                .UNSTABLE_addTemporalMarkerOffset(1.2, () -> bot.target = 0)
                .lineTo(new Vector2d(36, 45))
                .splineToConstantHeading(new Vector2d(60, 60), Math.toRadians(0))
//                .lineTo(new Vector2d(34, 18))
//                .splineToConstantHeading(new Vector2d(60, 5), Math.toRadians(0))
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
                    bot.target = 0;
                    bot.closeClaw();
                    bot.odometry.followTrajectorySequence(findBlueFProp);

                    if (bot.pipeline.isPropLeft()) {
                        spikeLoc = SPIKE_LOC.LEFT;
                        telemetry.addData("x", bot.pipeline.centerX);
                        if (bot.visionPortal.getCameraState() == VisionPortal.CameraState.STREAMING) {
                            bot.visionPortal.close();
                        }
                        break;
                    } else if (bot.pipeline.isPropCenter()) {
                        spikeLoc = SPIKE_LOC.CENTER;
                        telemetry.addData("x", bot.pipeline.centerX);
                        if (bot.visionPortal.getCameraState() == VisionPortal.CameraState.STREAMING) {
                            bot.visionPortal.close();
                        }
                        break;
                    } else {
                        spikeLoc = SPIKE_LOC.RIGHT;
                        telemetry.addData("x", bot.pipeline.centerX);
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