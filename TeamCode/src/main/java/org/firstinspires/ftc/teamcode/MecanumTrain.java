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

import androidx.annotation.NonNull;

import java.lang.Math;
import java.lang.Thread;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import com.arcrobotics.ftclib.controller.PIDController;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.drive.DriveSignal;
import com.acmerobotics.roadrunner.followers.HolonomicPIDVAFollower;
import com.acmerobotics.roadrunner.followers.TrajectoryFollower;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.*;
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.qualcomm.hardware.lynx.commands.core.LynxReadVersionStringResponse;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.StandardTrackingWheelLocalizer;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;

import static org.firstinspires.ftc.teamcode.drive.DriveConstants.*;

import org.firstinspires.ftc.teamcode.trajectorysequence.*;
import com.acmerobotics.roadrunner.drive.MecanumDrive;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
public class MecanumTrain extends MecanumDrive {
    // Public OpMode members.
    public DcMotorEx leftFrontDrive;
    public DcMotorEx leftBackDrive;
    public DcMotorEx rightFrontDrive;
    public DcMotorEx rightBackDrive;
    public DcMotorEx leftSlide;
    public DcMotorEx rightSlide;
    public DcMotorEx spinTake;
    public DcMotorEx outMotor;

    public Servo claw;
    public Servo drone;

    public OpenCvCamera camera;
    public AprilTagDetectionPipeline aprilTagDetectionPipeline;

    HardwareMap hwMap = null;

    private VoltageSensor vSensor;
    // Motor Info
    static final double COUNTS_PER_REV = 537.6;
    static final double COUNTS_PER_INCH = (COUNTS_PER_REV * GEAR_RATIO) /
            (2 * WHEEL_RADIUS * Math.PI);
    public int target = 0;

    private PIDController controllerArm;
    private PIDController controllerLift;
    public static double p = 0.02, i = 0, d = 0.0001, f = 0.12;
    public static double p_lift = 0, i_lift = 0, d_lift = 0, f_lift = 0;

    // Roadrunner Constants
    public static com.acmerobotics.roadrunner.control.PIDCoefficients TRANSLATIONAL_PID =
            new com.acmerobotics.roadrunner.control.PIDCoefficients(0, 0, 0);
    public static com.acmerobotics.roadrunner.control.PIDCoefficients HEADING_PID =
            new com.acmerobotics.roadrunner.control.PIDCoefficients(0, 0, 0);

    public static double LATERAL_MULTIPLIER = 1;

    public static double VX_WEIGHT = 1;
    public static double VY_WEIGHT = 1;
    public static double OMEGA_WEIGHT = 1;

    private TrajectorySequenceRunner trajectorySequenceRunner;

    private static final TrajectoryVelocityConstraint VEL_CONSTRAINT = getVelocityConstraint(MAX_VEL, MAX_ANG_VEL, TRACK_WIDTH);
    private static final TrajectoryAccelerationConstraint ACCEL_CONSTRAINT = getAccelerationConstraint(MAX_ACCEL);

    private TrajectoryFollower follower;

    private List<Integer> lastEncPositions = new ArrayList<>();
    private List<Integer> lastEncVels = new ArrayList<>();
    private List<DcMotorEx> motors;


    public double arm_start;
    public int liftL_start;
    public int liftR_start;

    public static double CLAW_OPEN = 0.2;
    public static double CLAW_CLOSED = 0;

    public static double DRONE_OPEN = .5;
    public static double DRONE_CLOSED = 1;

    public static int liftL_speed = 500;
    public static int liftR_speed = 450;

    public Gamepad.RumbleEffect rumbleEffect;

    public MecanumTrain(HardwareMap hwMapX, ElapsedTime runtime) {
        //Roadrunner Initialization
        super(kV, kA, kStatic, TRACK_WIDTH, TRACK_WIDTH, LATERAL_MULTIPLIER);

        hwMap = hwMapX; // saves reference to hwMap

        follower = new HolonomicPIDVAFollower(TRANSLATIONAL_PID, TRANSLATIONAL_PID, HEADING_PID,
                new Pose2d(0.5, 0.5, Math.toRadians(5.0)), 0.5);

        vSensor = hwMap.voltageSensor.iterator().next();

        // Initialize the hardware variables. Note that the strings used here
        // as parameters to 'get' must correspond to the names assigned during the robot
        // configuration step (using the FTC Robot Controller app).
        leftFrontDrive = hwMap.get(DcMotorEx.class, "FLdrive");
        leftBackDrive = hwMap.get(DcMotorEx.class, "BLdrive");
        rightFrontDrive = hwMap.get(DcMotorEx.class, "FRdrive");
        rightBackDrive = hwMap.get(DcMotorEx.class, "BRdrive");

        leftSlide = hwMap.get(DcMotorEx.class, "Lslide");
        rightSlide = hwMap.get(DcMotorEx.class, "Rslide");
        spinTake = hwMap.get(DcMotorEx.class, "Spintake");
        outMotor = hwMap.get(DcMotorEx.class, "Outtake");

        claw = hwMap.get(Servo.class, "claw");
        drone = hwMap.get(Servo.class, "drone");

        motors = Arrays.asList(leftFrontDrive, leftBackDrive, rightFrontDrive, rightBackDrive);

        // Camera Initialization
        int cameraMonitorViewId = hwMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id",
                hwMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hwMap.get(WebcamName.class, "camera"),
                cameraMonitorViewId);

        setMotorsMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        outMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        setLocalizer(new StandardTrackingWheelLocalizer(hwMap, lastEncPositions, lastEncVels));

        leftBackDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        leftSlide.setDirection(DcMotorSimple.Direction.REVERSE);

        setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        leftSlide.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rightSlide.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        outMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        List<Integer> lastTrackingEncPositions = new ArrayList<>();
        List<Integer> lastTrackingEncVels = new ArrayList<>();

        trajectorySequenceRunner = new TrajectorySequenceRunner(
                follower, HEADING_PID, vSensor,
                lastEncPositions, lastEncVels, lastTrackingEncPositions, lastTrackingEncVels
        );

        controllerArm = new PIDController(p, i, d);

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
        double denominator = Math.max(Math.abs(axial) + Math.abs(lateral) + Math.abs(yaw), 1);
        motorPowers[0] = (axial - lateral + yaw) / (denominator * 1.2);
        motorPowers[1] = (axial + lateral + yaw) / (denominator * 1.2);
        motorPowers[2] = (axial + lateral - yaw) / (denominator * 1.2);
        motorPowers[3] = (axial - lateral - yaw) / (denominator * 1.2);
        return motorPowers;
    }

    public void setMotorPowers(double v, double v1, double v2, double v3) {
        leftBackDrive.setPower(v);
        leftFrontDrive.setPower(v1);
        rightBackDrive.setPower(v2);
        rightFrontDrive.setPower(v3);

    }

    public void runIntake(double power) { spinTake.setPower(power); }

    public void runLift (int pos) {
        leftSlide.setTargetPosition((pos + liftL_start));
        leftSlide.setVelocity(liftL_speed);
        leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightSlide.setTargetPosition(pos + liftR_start);
        rightSlide.setVelocity(liftR_speed);
        rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void runOuttake(double power) { outMotor.setPower(power); }

    public void closeClaw () { claw.setPosition(CLAW_CLOSED); }
    public void openClaw () { claw.setPosition(CLAW_OPEN); }

    public void closeDrone() { drone.setPosition(DRONE_CLOSED); }
    public void openDrone() { drone.setPosition(DRONE_OPEN); }

    public void trainStop() {
        setMotorPowers(0, 0, 0, 0);
        leftSlide.setPower(0.0);
        rightSlide.setPower(0.0);
        outMotor.setPower(0.0);
        spinTake.setPower(0.0);
    }

    public void updateArmPID(Telemetry telemetry, double armPos) {
        controllerArm.setPID(p, i, d);
        double pid = controllerArm.calculate(armPos, target + arm_start);
        telemetry.addData("pid", pid);

        double ff =  Math.cos(Math.toRadians((target + arm_start) / (COUNTS_PER_REV / (2 * Math.PI)))) * f;
        double power = pid + ff;

        outMotor.setPower(power);
    }

    public double updateLiftPID() {
        controllerLift.setPID(p_lift, i_lift, d_lift);
        int liftPos = leftSlide.getCurrentPosition();
        double pid = controllerLift.calculate(liftPos, target);

        double power = pid;
        return power;
    }

    @NonNull
    @Override
    public List<Double> getWheelPositions() {
        lastEncPositions.clear();

        List<Double> wheelPositions = new ArrayList<>();
        for (DcMotorEx motor : motors) {
            int position = motor.getCurrentPosition();
            lastEncPositions.add(position);
            wheelPositions.add(encoderTicksToInches(position));
        }
        return wheelPositions;
    }

    @Override
    public List<Double> getWheelVelocities() {
        lastEncVels.clear();

        List<Double> wheelVelocities = new ArrayList<>();
        for (DcMotorEx motor : motors) {
            int vel = (int) motor.getVelocity();
            lastEncVels.add(vel);
            wheelVelocities.add(encoderTicksToInches(vel));
        }
        return wheelVelocities;
    }

    public TrajectoryBuilder trajectoryBuilder (Pose2d startPose) {
        return new TrajectoryBuilder(startPose, VEL_CONSTRAINT, ACCEL_CONSTRAINT);
    }

    public TrajectoryBuilder trajectoryBuilder (Pose2d startPose, boolean reversed) {
        return new TrajectoryBuilder(startPose, reversed, VEL_CONSTRAINT, ACCEL_CONSTRAINT);
    }

    public TrajectoryBuilder trajectoryBuilder (Pose2d startPose, double startHeading) {
        return new TrajectoryBuilder(startPose, startHeading, VEL_CONSTRAINT, ACCEL_CONSTRAINT);
    }

    public TrajectorySequenceBuilder trajectorySequenceBuilder(Pose2d startPose) {
        return new TrajectorySequenceBuilder(startPose, VEL_CONSTRAINT, ACCEL_CONSTRAINT,
                                                MAX_ANG_VEL, MAX_ANG_ACCEL);
    }

    public static TrajectoryVelocityConstraint getVelocityConstraint(double maxVel, double maxAngularVel, double trackWidth) {
        return new MinVelocityConstraint(Arrays.asList(
                new AngularVelocityConstraint(maxAngularVel),
                new MecanumVelocityConstraint(maxVel, trackWidth)
        ));
    }

    public void turnAsync(double angle) {
        trajectorySequenceRunner.followTrajectorySequenceAsync(
                trajectorySequenceBuilder(getPoseEstimate())
                        .turn(angle)
                        .build()
        );
    }

    public void turn(double angle) {
        turnAsync(angle);
        waitForIdle();
    }

    public void update() {
        updatePoseEstimate();
        DriveSignal signal = trajectorySequenceRunner.update(getPoseEstimate(), getPoseVelocity());
        if (signal != null) setDriveSignal(signal);
    }

    public void waitForIdle() {
        while (!Thread.currentThread().isInterrupted() && isBusy())
            update();
    }

    public static TrajectoryAccelerationConstraint getAccelerationConstraint(double maxAccel) {
        return new ProfileAccelerationConstraint(maxAccel);
    }

    public boolean isBusy() {
        return trajectorySequenceRunner.isBusy();
    }

    public void followTrajectoryAsync(Trajectory trajectory) {
        trajectorySequenceRunner.followTrajectorySequenceAsync(
                trajectorySequenceBuilder(trajectory.start())
                        .addTrajectory(trajectory)
                        .build()
        );
    }

    public void followTrajectory(Trajectory trajectory) {
        followTrajectoryAsync(trajectory);
        waitForIdle();
    }

    public void setMotorsMode(DcMotorEx.RunMode mode) {
        for (DcMotorEx motor : motors) {
            motor.setMode(mode);
        }
    }

    public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior zpb) {
        for (DcMotorEx motor : motors) {
            motor.setZeroPowerBehavior(zpb);
        }
    }

    public double getRawExternalHeading() { return 0; }

    public int inchesToPosition (double inches) {
        return (int) (inches * COUNTS_PER_INCH);
    }

}
