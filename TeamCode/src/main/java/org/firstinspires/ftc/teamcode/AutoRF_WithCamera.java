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
import org.firstinspires.ftc.teamcode.TrajectoryLibrary;

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

        // Create TrajectoryLibrary to de-clutter OpMode
        TrajectoryLibrary trajLib = new TrajectoryLibrary(bot, startPose);

        bot.closeClaw();
        SPIKE_LOC spikeLoc = SPIKE_LOC.IDLE;
        waitForStart();
        double x = 0;

        while(opModeInInit()) {
            bot.initTfodRed(hardwareMap);
            bot.visionPortal.stopStreaming();
            bot.visionPortal.setProcessorEnabled(bot.tfod, false); // Disable TFOD to save CPU
        }
        while (opModeIsActive()) {
            while (runtime.seconds() < 4) {
                bot.visionPortal.resumeStreaming();
                bot.visionPortal.setProcessorEnabled(bot.tfod, true); // Re-enable TFOD
                bot.closeClaw();
                bot.odometry.followTrajectory(trajLib.findRedFProp);

                List<Recognition> currentRecognitions = bot.tfod.getRecognitions();
                if (bot.tfod.getRecognitions() != null) {
                    for (Recognition recognition : currentRecognitions) {
                        x = (recognition.getLeft() + recognition.getRight()) / 2;
                        telemetry.addData("x", x);
                    }
                    // Changed 1/13/2024 @ 2:43 PM
                    if (x > 550) {
                        spikeLoc = SPIKE_LOC.RIGHT;
                        break;
                    } else if (x < 550 && x > 200) {
                        spikeLoc = SPIKE_LOC.CENTER;
                        break;
                    } else {
                        spikeLoc = SPIKE_LOC.LEFT;
                        break;
                    }
                }
            }

            bot.visionPortal.stopStreaming();
            bot.visionPortal.setProcessorEnabled(bot.tfod, false); // Disable TFOD to save CPU

            switch (spikeLoc) {
                case RIGHT:
                    telemetry.addLine("RIGHT SPIKE");
                    if (!bot.odometry.isBusy()) {
                        bot.odometry.followTrajectorySequenceAsync(trajLib.RFdriveToRSpike);
                        spikeLoc = SPIKE_LOC.IDLE;
                        break;
                    }
                case CENTER:
                    telemetry.addLine("CENTER SPIKE");
                    if (!bot.odometry.isBusy()) {
                        bot.odometry.followTrajectorySequenceAsync(trajLib.RFdriveToCSpike);
                        spikeLoc = SPIKE_LOC.IDLE;
                        break;
                    }
                default:
                    telemetry.addLine("LEFT SPIKE");
                    if (!bot.odometry.isBusy()) {
                        bot.odometry.followTrajectorySequenceAsync(trajLib.RFdriveToLSpike);
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
