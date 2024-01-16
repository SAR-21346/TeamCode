package org.firstinspires.ftc.teamcode;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.vision.VisionPortal;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.List;

@Autonomous(name = "Autonomous BlueFront With Camera")
public class AutoBF_WithCamera extends LinearOpMode {
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

        Pose2d startPose = new Pose2d(10.5, 59, Math.toRadians(0));
        bot.odometry.setPoseEstimate(startPose);

        // Create TrajectoryLibrary to de-clutter OpMode
        TrajectoryLibrary trajLib = new TrajectoryLibrary(bot, startPose);


        bot.closeClaw();
        SPIKE_LOC spikeLoc = SPIKE_LOC.IDLE;
        waitForStart();
        double x = 0;

        while(opModeInInit()) {
            bot.initEocvBlue(hardwareMap);
            bot.visionPortal.stopStreaming();
            bot.visionPortal.setProcessorEnabled(bot.pipeline, false); // Disable TFOD to save CPU
        }

        while (opModeIsActive()) {
            while (runtime.seconds() < 4) {
                bot.visionPortal.resumeStreaming();
                bot.visionPortal.setProcessorEnabled(bot.pipeline, true); // Re-enable TFOD
                bot.closeClaw();
                bot.odometry.followTrajectory(trajLib.findBlueFProp);

                if (bot.pipeline.isPropRight()){
                    spikeLoc = SPIKE_LOC.RIGHT;
                } else if (bot.pipeline.isPropCenter()){
                    spikeLoc = SPIKE_LOC.CENTER;
                } else if (!(bot.pipeline.isPropCenter() && !bot.pipeline.isPropRight())) {
                    spikeLoc = SPIKE_LOC.LEFT;
                }
                telemetry.addData("x", bot.pipeline.centerX);
            }

            bot.visionPortal.stopStreaming();
            bot.visionPortal.setProcessorEnabled(bot.tfod, false); // Disable TFOD to save CPU

            switch (spikeLoc) {
                case RIGHT:
                    telemetry.addLine("RIGHT SPIKE");
                    if (!bot.odometry.isBusy()) {
                        bot.odometry.followTrajectorySequenceAsync(trajLib.BFdriveToRSpike);
                        spikeLoc = SPIKE_LOC.IDLE;
                        break;
                    }
                case CENTER:
                    telemetry.addLine("CENTER SPIKE");
                    if (!bot.odometry.isBusy()) {
                        bot.odometry.followTrajectorySequenceAsync(trajLib.BFdriveToCSpike);
                        spikeLoc = SPIKE_LOC.IDLE;
                        break;
                    }
                case LEFT:
                    telemetry.addLine("LEFT SPIKE");
                    if (!bot.odometry.isBusy()) {
                        bot.odometry.followTrajectorySequenceAsync(trajLib.BFdriveToLSpike);
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