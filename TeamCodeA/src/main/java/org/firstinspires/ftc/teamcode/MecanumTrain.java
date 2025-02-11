package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.RobotConstants.LEFT_DROPDOWN_MAX;
import static org.firstinspires.ftc.teamcode.RobotConstants.LEFT_DROPDOWN_MIN;
import static org.firstinspires.ftc.teamcode.RobotConstants.LEFT_EXT_MAX;
import static org.firstinspires.ftc.teamcode.RobotConstants.LEFT_EXT_MIN;
import static org.firstinspires.ftc.teamcode.RobotConstants.RIGHT_DROPDOWN_MAX;
import static org.firstinspires.ftc.teamcode.RobotConstants.RIGHT_DROPDOWN_MIN;
import static org.firstinspires.ftc.teamcode.RobotConstants.RIGHT_EXT_MAX;
import static org.firstinspires.ftc.teamcode.RobotConstants.RIGHT_EXT_MIN;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.pedropathing.util.Constants;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
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

    public double extLPos, extRPos, dropdownLPos, dropdownRPos,
            clawPos, hangPivotPos, outtakeFlipLPos, outtakeFlipRPos;

    public ColorSensor intakeWheel, intakeWall;
    public double intakeWallDist, intakeWheelDist;


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
        dropdownR = hwMap.get(Servo.class, "dropdownR");

        // ----------------- Sensors -----------------
        extEncL = hwMap.get(AnalogInput.class, "extEncL");
        extEncR = hwMap.get(AnalogInput.class, "extEncR");
        dropdownEncL = hwMap.get(AnalogInput.class, "dropdownEncL");
        dropdownEncR = hwMap.get(AnalogInput.class, "dropdownEncR");

        intakeWheel = hwMap.get(ColorSensor.class, "intakeWheel");
        intakeWall = hwMap.get(ColorSensor.class, "intakeWall");

        // Set ZeroPowerBehavior for Motors
        setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);

        // Set Modes for Motors
        setMotorsMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Set BulkCachingMode
        setBulkCachingMode();

        setDriveMotorDirections();
        intake.setDirection(DcMotorSimple.Direction.REVERSE);
        liftL.setDirection(DcMotorSimple.Direction.REVERSE);
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

    public void encoderUpdate() {
        extLPos = extEncL.getVoltage() / 3.3 * 360;
        extRPos = extEncR.getVoltage() / 3.3 * 360;
    }

    public void distSensorUpdate () {
        if (intakeWall instanceof DistanceSensor) {
            ColorSensor color = intakeWall;
            intakeWallDist = ((DistanceSensor) color).getDistance(DistanceUnit.MM);
        }

        if (intakeWheel instanceof DistanceSensor) {
            ColorSensor color = intakeWheel;
            intakeWheelDist = ((DistanceSensor) color).getDistance(DistanceUnit.MM);
        }
    }

    public void pivot_down() {
        dropdownL.setPosition(LEFT_DROPDOWN_MAX);
        dropdownR.setPosition(RIGHT_DROPDOWN_MAX);
    }

    public void pivot_up() {
        dropdownL.setPosition(LEFT_DROPDOWN_MIN);
        dropdownR.setPosition(RIGHT_DROPDOWN_MIN);
    }

    public void extend (int distance) {
        switch (distance) {
            case 1:
                extL.setPosition(LEFT_EXT_MAX*0.2);
                extR.setPosition(RIGHT_EXT_MAX*0.2);
                break;
            case 2:
                extL.setPosition(LEFT_EXT_MAX*0.4);
                extR.setPosition(RIGHT_EXT_MAX*0.4);
                break;
            case 3:
                extL.setPosition(LEFT_EXT_MAX*0.6);
                extR.setPosition(RIGHT_EXT_MAX*0.6);
                break;
            case 4:
                extL.setPosition(LEFT_EXT_MAX*0.8);
                extR.setPosition(RIGHT_EXT_MAX*0.8);
                break;
            case 5:
                extL.setPosition(LEFT_EXT_MAX);
                extR.setPosition(RIGHT_EXT_MAX);
                break;
        }
    }

    public void retract() {
        extL.setPosition(LEFT_EXT_MIN);
        extR.setPosition(RIGHT_EXT_MIN);
    }

    // intakeWheelDetect()
    // Detects the color of the object in front of the intake wheel
    // returns 1 for red, 2 for blue, 3 for yellow
    public int intakeWheelDetect() {
        int r = intakeWheel.red(), g = intakeWheel.green(), b = intakeWheel.blue();
        int maxValue = Math.max(r, Math.max(g, b));
        if (maxValue == r) {
            return 1;
        } else if (maxValue == b) {
            return 2;
        } else {
            return 3;
        }
    }
}
