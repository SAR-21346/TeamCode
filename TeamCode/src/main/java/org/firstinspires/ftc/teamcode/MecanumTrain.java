/* Copyright (c) 2021 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;


import java.lang.Math;
import java.lang.Thread;
import java.util.Arrays;
import java.util.List;


import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.StandardTrackingWheelLocalizer;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;

import static org.firstinspires.ftc.teamcode.drive.DriveConstants.*;

import android.util.Size;

import org.firstinspires.ftc.teamcode.trajectorysequence.*;
import com.acmerobotics.roadrunner.drive.MecanumDrive;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
public class MecanumTrain{
    // Drive Motors
    public DcMotorEx leftFrontDrive;
    public DcMotorEx leftBackDrive;
    public DcMotorEx rightFrontDrive;
    public DcMotorEx rightBackDrive;

    // Auxiliary Motors
    public DcMotorEx leftSlide;
    public DcMotorEx rightSlide;
    public DcMotorEx spinTake;
    public DcMotorEx outMotor;

    // Servos
    public Servo claw;
    public Servo drone;

    // Odometry helper class
    public SampleMecanumDrive odometry;

    // Camera
    public AprilTagProcessor aprilTagProcessor;
    public TfodProcessor tfod;
    public VisionPortal visionPortal;

    // Hardware Map
    HardwareMap hwMap = null;

    // Motor Info
    static final double COUNTS_PER_REV = 537.6;
    static final double COUNTS_PER_INCH = (COUNTS_PER_REV * GEAR_RATIO) /
            (2 * WHEEL_RADIUS * Math.PI);
    public int target = 0;

    private PIDController controllerArm;
    private PIDController controllerLift;
    public static double p = 0.02, i = 0, d = 0.0001, f = 0.12;
    public static double p_lift = 0, i_lift = 0, d_lift = 0, f_lift = 0;

    private List<DcMotorEx> motors;

    // Initial Motor Positions
    public double arm_start;
    public int liftL_start;
    public int liftR_start;

    // Claw and Drone Position Constants
    public static double CLAW_OPEN = 0.2;
    public static double CLAW_CLOSED = 0;

    public static double DRONE_OPEN = .5;
    public static double DRONE_CLOSED = 1;

    // Lift Speeds
    public static int liftL_speed = 500;
    public static int liftR_speed = 500;

    public Gamepad.RumbleEffect rumbleEffect;

    public MecanumTrain(HardwareMap hwMapX, ElapsedTime runtime) {
        hwMap = hwMapX; // saves reference to hwMap

        // Odometry
        odometry = new SampleMecanumDrive(hwMap);

        // Drive Motors
        leftFrontDrive = hwMap.get(DcMotorEx.class, "FLdrive");
        leftBackDrive = hwMap.get(DcMotorEx.class, "BLdrive");
        rightFrontDrive = hwMap.get(DcMotorEx.class, "FRdrive");
        rightBackDrive = hwMap.get(DcMotorEx.class, "BRdrive");

        // Auxiliary Motors
        leftSlide = hwMap.get(DcMotorEx.class, "Lslide");
        rightSlide = hwMap.get(DcMotorEx.class, "Rslide");
        spinTake = hwMap.get(DcMotorEx.class, "Spintake");
        outMotor = hwMap.get(DcMotorEx.class, "Outtake");

        // Servos
        claw = hwMap.get(Servo.class, "claw");
        drone = hwMap.get(Servo.class, "drone");

        // Motor List
        motors = Arrays.asList(leftFrontDrive, leftBackDrive, rightFrontDrive, rightBackDrive);

        // Set Modes for Motors
        setMotorsMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        outMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // Set directions for Motors
        leftBackDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        leftSlide.setDirection(DcMotorSimple.Direction.REVERSE);

        // Set ZeroPowerBehavior for Motors
        setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        leftSlide.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rightSlide.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        outMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        // Instantiate PID Controllers
        controllerArm = new PIDController(p, i, d);
        controllerLift = new PIDController(p_lift, i_lift, d_lift);

        // Gamepad Rumble Effect
        rumbleEffect = new Gamepad.RumbleEffect.Builder()
                .addStep(0.5, 0.5, 500)
                .addStep(0.0,0.0, 300)
                .addStep(1.0, 1.0, 500)
                .addStep(0.0, 0.0, 1000)
                .build();
        }

    // calculateMotorPowers(axial, lateral, yaw)
    // axial - double
    // lateral - double
    // yaw - double
    // returns double[]
    public double[] calculateMotorPowers(double axial, double lateral, double yaw) {
        double[] motorPowers = new double[4];
        motorPowers[0] = axial + lateral + yaw;
        motorPowers[1] = axial - lateral - yaw;
        motorPowers[2] = axial - lateral + yaw;
        motorPowers[3] = axial + lateral - yaw;
        return motorPowers;
    }

    // setMotorPowers(v, v1, v2, v3)
    // v - double (power for leftBackDrive)
    // v1 - double (power for leftFrontDrive)
    // v2 - double (power for rightBackDrive)
    // v3 - double (power for rightFrontDrive)
    public void setMotorPowers(double v, double v1, double v2, double v3) {
        leftBackDrive.setPower(v2);
        leftFrontDrive.setPower(v);
        rightBackDrive.setPower(v3);
        rightFrontDrive.setPower(v1);
    }

    // runIntake(power)
    // power - double (power for spinTake)
    public void runIntake(double power) { spinTake.setPower(power); }

    // runLift(pos)
    // pos - int (target position for leftSlide and rightSlide)
    public void runLift (int pos) {
        leftSlide.setTargetPosition((pos + liftL_start)); // liftL_start is the initial position of the left slide
        leftSlide.setVelocity(liftL_speed); // Was told to change speeds for the individual motors
        leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION); // Allows motor to move via encoder
        rightSlide.setTargetPosition(pos + liftR_start); // liftR_start is the initial position of the right slide
        rightSlide.setVelocity(liftR_speed); // Was told to change speeds for the individual motors
        rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION); // Allows motor to move via encoder
    }

    // runOuttake(power)
    // power - double (power for outMotor)
    public void runOuttake(double power) { outMotor.setPower(power); }

    public void openClaw() { claw.setPosition(CLAW_OPEN); }
    public void closeClaw() { claw.setPosition(CLAW_CLOSED); }
    public void openDrone() { drone.setPosition(DRONE_OPEN); }
    public void closeDrone() { drone.setPosition(DRONE_CLOSED); }

    // update()
    // Updates the telemetry with the current encoder values for the arm
    // Should be called in a loop
    public void updateArmPID(double armPos) {
        controllerArm.setPID(p, i, d);
        double pid = controllerArm.calculate(armPos, target + arm_start);
        double ff =  Math.cos(Math.toRadians((target + arm_start) / (COUNTS_PER_REV / (2 * Math.PI)))) * f;
        double power = pid + ff;
        outMotor.setPower(power);
    }

    // setMotorsMode(mode)
    // mode - DcMotor.RunMode (run mode for all motors)
    // Iterates through all drive motors to change their run mode
    public void setMotorsMode(DcMotorEx.RunMode mode) {
        for (DcMotorEx motor : motors) {
            motor.setMode(mode);
        }
    }

    // setZeroPowerBehavior(zpb)
    // zpb - DcMotor.ZeroPowerBehavior (zero power behavior for all motors)
    // Iterates through all drive motors to change their zero power behavior (BRAKE or FLOAT)
    public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior zpb) {
        for (DcMotorEx motor : motors) {
            motor.setZeroPowerBehavior(zpb);
        }
    }

    public void initTfodRed(HardwareMap hwMap) {
        final String[] LABELS = {"RedProp1"};
        tfod = new TfodProcessor.Builder()
                .setModelFileName("/sdcard/FIRST/tflitemodels/RedProp1.tflite")
                .setModelLabels(LABELS)
                .build();
        VisionPortal.Builder builder = new VisionPortal.Builder();
        builder.setCamera(hwMap.get(WebcamName.class, "camera"));
        builder.setCameraResolution(new Size(800, 448));
        builder.addProcessor(tfod);
        visionPortal = builder.build();
        tfod.setMinResultConfidence(0.4f);
    }

    public void initTfodBlue(HardwareMap hwMap) {
        tfod = new TfodProcessor.Builder()
                .setModelFileName("/sdcard/FIRST/tflitemodels/BlueProp1.tflite")
                .setModelLabels(new String[]{"BlueProp1"})
                .build();
        VisionPortal.Builder builder = new VisionPortal.Builder();
        builder.setCamera(hwMap.get(WebcamName.class, "camera"));
        builder.setCameraResolution(new Size(800, 448));
        builder.addProcessor(tfod);
        visionPortal = builder.build();
        tfod.setMinResultConfidence(0.4f);
    }

    public void tfodRecRed() {

    }

}
