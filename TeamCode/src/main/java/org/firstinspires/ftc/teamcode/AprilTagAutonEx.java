//NOTE! Currently only setup for Red Side, if approved, will work on blue.

package org.firstinspires.ftc.teamcode;

import android.annotation.SuppressLint;

import org.firstinspires.ftc.teamcode.AprilTagDetectionPipeline;
import org.firstinspires.ftc.teamcode.MecanumTrain;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;


import java.util.ArrayList;


@Autonomous(name = "AprilTag Autonomous")
public class AprilTagAutonEx extends LinearOpMode {

    MecanumTrain bot;


    static final double FEET_PER_METER = 3.28084;

    // Lens intrinsics
    // UNITS ARE PIXELS

    // DO NOT TOUCH THIS PART UNLESS YOU KNOW WHAT YOU ARE DOING!
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

    // UNITS ARE METERS
    double tagsize = 0.166;

    int LEFT_RED = 4;
    int MID_RED = 5;
    int RIGHT_RED = 6;

    AprilTagDetection tagOfInterest = null; // this is the tag we're interested in

    // Currently set to tags stated in gm2

    @Override
    public void runOpMode() {
        bot = new MecanumTrain(); // this is the robot instance
        bot.init(hardwareMap);

        AprilTagDetectionPipeline aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);
        bot.camera.setPipeline(aprilTagDetectionPipeline);

        bot.camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                bot.camera.startStreaming(800, 448, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                telemetry.addLine("Camera error: " + errorCode);
                telemetry.update();
            }
        });

        telemetry.setMsTransmissionInterval(50);



        waitForStart();


        while (opModeIsActive()) {
            ArrayList<AprilTagDetection> detections = aprilTagDetectionPipeline.getLatestDetections();

            if (detections.size() != 0) {
                boolean tagFoundLR = false;
                boolean tagFoundMR = false;
                boolean tagFoundRR = false;

                for (AprilTagDetection tag : detections) {
                    if (tag.id == LEFT_RED) {
                        tagOfInterest = tag;
                        tagFoundLR = true;
                        break;
                    } else if (tag.id == MID_RED) {
                        tagOfInterest = tag;
                        tagFoundMR = true;
                        break;
                    } else if (tag.id == RIGHT_RED) {
                        tagOfInterest = tag;
                        tagFoundRR = true;
                        break;
                    }
                }

                if (tagFoundLR) {
                    telemetry.addLine("Left Tag Detected\n Red Alliance\nLocation data:");
                    tagToTelemetry(tagOfInterest);
                } else if (tagFoundMR) {
                    telemetry.addLine("Middle Tag Detected\n Red Alliance\nLocation data:");
                    tagToTelemetry(tagOfInterest);
                } else if (tagFoundRR) {
                    telemetry.addLine("Right Tag Detected\n Red Alliance\nLocation data:");
                    tagToTelemetry(tagOfInterest);
                }
            } else {
                telemetry.addLine("Don't see tag of interest :(");

                if (tagOfInterest == null) {
                    telemetry.addLine("(The tag has never been seen)");
                } else {
                    telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                    tagToTelemetry(tagOfInterest);
                }

            }

            telemetry.update();
            sleep(20);
        }

        /*
         * The START command just came in: now work off the latest snapshot acquired
         * during the init loop.
         */

        /* Update the telemetry */
        if (tagOfInterest != null) {
            telemetry.addLine("Tag snapshot:\n");
            tagToTelemetry(tagOfInterest);
            telemetry.update();
        } else {
            telemetry.addLine("No tag snapshot available, it was never sighted during the init loop :(");
            telemetry.update();
        }

        /* Actually do something useful */
        if (tagOfInterest == null) {
            /*
             * Insert your autonomous code here, presumably running some default
             * configuration
             * since the tag was never sighted during INIT
             */
        } else {
            /*
             * Insert your autonomous code here, probably using the tag to decide your
             * configuration.
             */

            // e.g.
//            if (tagFoundLR) {
//                // do something
//                telemetry.addLine("Red Left Tag path running");
//            } else if (tagFoundMR) {
//                // do something else
//                telemetry.addLine("Red Mid Tag path running");
//            } else if (tagFoundRR) {
//                // do something else
//                telemetry.addLine("Red Right Tag path running");
//            }
        }

    }


    void tagToTelemetry(AprilTagDetection detection) {
        Orientation rot = Orientation.getOrientation(detection.pose.R, AxesReference.INTRINSIC, AxesOrder.YXZ,
                AngleUnit.DEGREES);

        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id)); // tag id
        telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x * FEET_PER_METER)); // x position
        telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y * FEET_PER_METER)); // y position
        telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z * FEET_PER_METER)); // z position
        telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", rot.firstAngle)); // yaw rotation
        telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", rot.secondAngle)); // pitch rotation
        telemetry.addLine(String.format("Rotation Roll: %.2f degrees", rot.thirdAngle)); // roll rotation
    }
}

// This might be pretty buggy, and I think it works primarily through the init
// cycle, but it should work in other parts as well

// this was also heavily reused from last year sooooooo...
// this is because u took this from the eocv thing and didn't change stuff that
// was necessary for it to work lol