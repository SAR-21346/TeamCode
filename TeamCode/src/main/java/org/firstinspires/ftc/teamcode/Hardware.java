package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.qualcomm.hardware.bosch.BHI260IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotor.RunMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.MotorControlAlgorithm;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.openftc.easyopencv.OpenCvCamera;

public abstract class Hardware extends LinearOpMode {
    public DcMotorEx leftFrontDrive;
    public DcMotorEx leftBackDrive;
    public DcMotorEx rightFrontDrive;
    public DcMotorEx rightBackDrive;
    public DcMotorEx leftSlide;
    public DcMotorEx rightSlide;
    public DcMotorEx spinTake;
    public DcMotorEx outMotor;

    public Servo clawAuto;
    public Servo drone;

    public OpenCvCamera camera;
    public AprilTagDetectionPipeline aprilTagDetectionPipeline;
    public void initHardware() {
        // Initialize motors
        leftFrontDrive = (DcMotorEx) hardwareMap.dcMotor.get("FLdrive");
        rightFrontDrive = (DcMotorEx) hardwareMap.dcMotor.get("FRdrive");
        leftBackDrive = (DcMotorEx) hardwareMap.dcMotor.get("BLdrive");
        rightBackDrive = (DcMotorEx) hardwareMap.dcMotor.get("BRdrive");
        clawAuto = hardwareMap.get(Servo.class, "claw");


        // Initialize BHI260AP sensor
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        waitForStart();
    }

    // stop moving
    public void stopMoving() {
        rightFrontDrive.setPower(0);
        leftFrontDrive.setPower(0);
        rightBackDrive.setPower(0);
        leftBackDrive.setPower(0);
    }

    // We can have the go backwards be negative power
    // forward -> use positive power
    // backward -> use negative power
    public void moveY(double power) {
        rightFrontDrive.setPower(power);
        leftFrontDrive.setPower(power);
        rightBackDrive.setPower(power);
        leftBackDrive.setPower(power);
    }

    // its the same thing as moveY. Positive power is going to be right and negative is left
    // right -> positive power
    // left -> negative power
    public void moveX(double power) {
        rightFrontDrive.setPower(-power);
        leftFrontDrive.setPower(power);
        rightBackDrive.setPower(power);
        leftBackDrive.setPower(-power);
    }

}