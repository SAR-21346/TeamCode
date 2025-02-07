package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.pedropathing.util.Constants;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants;

import java.util.Arrays;
import java.util.List;

@Config
public class MecanumTrain{
    HardwareMap hwMap; // saves HardwareMap reference to hwMap

    // ----------------- Drive Motors -----------------
    public DcMotorEx leftFrontDrive, leftBackDrive, rightFrontDrive, rightBackDrive;
    private final List<DcMotorEx> motors;

    // ----------------- Auxillary Motors -----------------
    public DcMotorEx liftL, liftR, intake;

    // ----------------- Servos -----------------
    public Servo extL, extR, dropdownL, dropdownR,
            hangL, hangR, claw, hangPivot, outtakeFlipL, outtakeFlipR;

    // ----------------- Odometry -----------------
    public Follower follower;
    public DcMotorEx leftEnc, rightEnc, strafeEnc;

    // ----------------- Sensors -----------------
    public AnalogInput extEncL, extEncR, dropdownEncL, dropdownEncR,
            clawEnc, hangPivotEnc, outtakeFlipEncL, outtakeFlipEncR;


    public MecanumTrain(HardwareMap hwMapX) {
        hwMap = hwMapX; // saves reference to hwMap

        // ----------------- Odometry -----------------
        leftEnc = hwMap.get(DcMotorEx.class, "backLeft");
        rightEnc = hwMap.get(DcMotorEx.class, "parR");
        strafeEnc = hwMap.get(DcMotorEx.class, "backRight");
        leftEnc.setDirection(DcMotorSimple.Direction.REVERSE);
        rightEnc.setDirection(DcMotorSimple.Direction.FORWARD);
        strafeEnc.setDirection(DcMotorSimple.Direction.FORWARD);

        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hwMap);


        // ----------------- Drive Motors -----------------
        leftFrontDrive = hwMap.get(DcMotorEx.class, "frontLeft");
        rightFrontDrive = hwMap.get(DcMotorEx.class, "frontRight");
        leftBackDrive = hwMap.get(DcMotorEx.class, "backLeft");
        rightBackDrive = hwMap.get(DcMotorEx.class, "backRight");

        motors = Arrays.asList(leftFrontDrive, leftBackDrive, rightFrontDrive, rightBackDrive);

        // ----------------- Auxillary Motors -----------------
        liftL = hwMap.get(DcMotorEx.class, "liftL");
        liftR = hwMap.get(DcMotorEx.class, "liftR");
        intake = hwMap.get(DcMotorEx.class, "intake");

        // ----------------- Servos -----------------
        extL = hwMap.get(Servo.class, "extL");
        extR = hwMap.get(Servo.class, "extR");
        dropdownL = hwMap.get(Servo.class, "dropdownL");

        // ----------------- Sensors -----------------
        extEncL = hwMap.get(AnalogInput.class, "extEncL");
        extEncR = hwMap.get(AnalogInput.class, "extEncR");

        // Set ZeroPowerBehavior for Motors
        setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);

        // Set Modes for Motors
        setMotorsMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Set BulkCachingMode
        setBulkCachingMode();

        setDriveMotorDirections();
        }

    // calculateMotorPowers(axial, lateral, yaw)
    // axial - double
    // lateral - double
    // yaw - double
    // returns double[]
    public double[] calculateMotorPowers(double axial, double lateral, double yaw) {
        double[] motorPowers = new double[4];
        double multiplier = 1;
        motorPowers[0] = (axial + lateral + yaw) * multiplier; // front left
        motorPowers[1] = (axial - lateral + yaw) * multiplier; // back left
        motorPowers[2] = (axial - lateral - yaw) * multiplier; // front right
        motorPowers[3] = (axial + lateral - yaw) * multiplier; // back right
        return motorPowers;
    }

    // setMotorPowers(v, v1, v2, v3)
    // v - double (power for leftBackDrive)
    // v1 - double (power for leftFrontDrive)
    // v2 - double (power for rightBackDrive)
    // v3 - double (power for rightFrontDrive)
    public void setMotorPowers(double v, double v1, double v2, double v3, double speedMultiplier) {
        leftFrontDrive.setPower(v * speedMultiplier);
        leftBackDrive.setPower(v1 * speedMultiplier);
        rightFrontDrive.setPower(v2 * speedMultiplier);
        rightBackDrive.setPower(v3 * speedMultiplier);
    }

    // setMotorsMode(mode)
    // mode - DcMotor.RunMode (run mode for all motors)
    // Iterates through all drive motors to change their run mode
    private void setMotorsMode(DcMotorEx.RunMode mode) {
        for (DcMotorEx motor : motors) {
            motor.setMode(mode);
        }
    }

    // setZeroPowerBehavior(zpb)
    // zpb - DcMotor.ZeroPowerBehavior (zero power behavior for all motors)
    // Iterates through all drive motors to change their zero power behavior (BRAKE or FLOAT)
    private void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior zpb) {
        for (DcMotorEx motor : motors) {
            motor.setZeroPowerBehavior(zpb);
        }
    }

    private void setBulkCachingMode () {
        // Set BulkCachingMode for all hubs - gets sensor reads faster
        List<LynxModule> allHubs = hwMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
    }

    private void setDriveMotorDirections() {
        leftFrontDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotorSimple.Direction.FORWARD);
    }
}
