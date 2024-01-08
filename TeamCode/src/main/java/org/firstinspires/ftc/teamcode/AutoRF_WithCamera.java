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
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

import java.util.List;

@Autonomous(name = "Autonomous RedFront With Camera")
public class AutoRF_WithCamera extends LinearOpMode {
    MecanumTrain bot;

    enum SPIKE_LOC  {
            LEFT,
            CENTER,
            RIGHT
    }
    SPIKE_LOC spikeLoc = SPIKE_LOC.LEFT;

    @Override
    public void runOpMode() throws InterruptedException {
        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        ElapsedTime runtime = new ElapsedTime();
        bot = new MecanumTrain(hardwareMap, runtime);

        bot.initTfodRed(hardwareMap);

        Pose2d startPose = new Pose2d(10.5, -59, Math.toRadians(180));
        bot.odometry.setPoseEstimate(startPose);

        Trajectory findRedProp = bot.odometry.trajectoryBuilder(startPose)
                .lineTo(new Vector2d(12, -53))
                .build();

        TrajectorySequence driveToRSpike = bot.odometry.trajectorySequenceBuilder(findRedProp.end())
                .splineToSplineHeading(new Pose2d(23, -46, Math.toRadians(90)), Math.toRadians(0))
                .lineTo(new Vector2d(23, -38))
                .addDisplacementMarker(() -> {bot.openClaw();})
                .waitSeconds(.5)
                .lineTo(new Vector2d(23, -40))
                .addDisplacementMarker(() -> {bot.closeClaw();})
                .waitSeconds(.5)
                .lineTo(new Vector2d(23, -46))
                .lineTo(new Vector2d(25, -46))
                .splineToSplineHeading(new Pose2d(48, -42, Math.toRadians(180)), Math.toRadians(0))
                .build();

        TrajectorySequence driveToCSpike = bot.odometry.trajectorySequenceBuilder(findRedProp.end())
                .splineToSplineHeading(new Pose2d(20.5, -27.5, Math.toRadians(150)), Math.toRadians(150))
                .addDisplacementMarker(() -> {bot.openClaw();})
                .waitSeconds(.5)
                .back(2)
                .addDisplacementMarker(() -> {bot.closeClaw();})
                .waitSeconds(.5)
                .splineToSplineHeading(new Pose2d(48, -35, Math.toRadians(180)), Math.toRadians(0))
                .build();

        TrajectorySequence driveToLSpike = bot.odometry.trajectorySequenceBuilder(findRedProp.end())
                .lineTo(new Vector2d(14, -44))
                .splineToSplineHeading(new Pose2d(4, -38.5, Math.toRadians(110)), Math.toRadians(180))
                .addDisplacementMarker(() -> {bot.openClaw();})
                .waitSeconds(0.5)
                .back(2)
                .addDisplacementMarker(() -> {bot.closeClaw();})
                .waitSeconds(0.5)
                .lineTo(new Vector2d(14, -44))
                .setConstraints(bot.odometry.SHAKE_VEL_CONSTRAINT, bot.odometry.SHAKE_ACCEL_CONSTRAINT)
                .forward(5)
                .back(5)
                .resetConstraints()
                .addDisplacementMarker(() -> {
                    bot.runLift(200);
                    bot.target = 150;
                })
                .splineToSplineHeading(new Pose2d(48, -28, Math.toRadians(180)), Math.toRadians(0))
                .addDisplacementMarker(() -> {bot.openClaw();})
                .waitSeconds(0.5)
                .addDisplacementMarker(() -> {
                    bot.closeClaw();
                    bot.target = 0;
                    bot.runLift(0);
                })
                .lineTo(new Vector2d(45, -15))
                .splineToConstantHeading(new Vector2d(60, -12.5), Math.toRadians(0))
                .build();

        waitForStart();

        double x = 0;

        while (opModeIsActive()) {
            while (runtime.seconds() < 3) {
                bot.odometry.followTrajectory(findRedProp);
            }

            List<Recognition> currentRecognitions = bot.tfod.getRecognitions();
            if (bot.tfod.getRecognitions() != null) {
                for (Recognition recognition : currentRecognitions) {
                    x = (recognition.getLeft() + recognition.getRight()) / 2;

                    if (x > 400) {
                        spikeLoc = SPIKE_LOC.RIGHT;
                    } else if (x < 400) {
                        spikeLoc = SPIKE_LOC.CENTER;
                    }

                    telemetry.addData("x", x);
                }
            }


            while(runtime.seconds() > 6) {
                switch (spikeLoc) {
                    case RIGHT:
                        telemetry.addLine("RIGHT SPIKE");
                        if (!bot.odometry.isBusy()) {
                            bot.odometry.followTrajectorySequenceAsync(driveToRSpike);
                        }
                        break;
                    case CENTER:
                        telemetry.addLine("CENTER SPIKE");
                        if (!bot.odometry.isBusy()) {
                            bot.odometry.followTrajectorySequenceAsync(driveToCSpike);
                        }
                        break;
                    case LEFT:
                        telemetry.addLine("LEFT SPIKE");
                        if (!bot.odometry.isBusy()) {
                            bot.odometry.followTrajectorySequenceAsync(driveToLSpike);
                        }
                        break;
                }
                bot.odometry.update();
                bot.updateArmPID(bot.outMotor.getCurrentPosition());
            }

            telemetry.update();

        }
    }
}
