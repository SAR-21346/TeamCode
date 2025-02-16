package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.RobotConstants.CLAW_MAX;
import static org.firstinspires.ftc.teamcode.RobotConstants.CLAW_MIN;
import static org.firstinspires.ftc.teamcode.RobotConstants.LEFT_DROPDOWN_MAX;
import static org.firstinspires.ftc.teamcode.RobotConstants.LEFT_DROPDOWN_MIN;
import static org.firstinspires.ftc.teamcode.RobotConstants.LEFT_EXT_MAX;
import static org.firstinspires.ftc.teamcode.RobotConstants.LEFT_EXT_MIN;
import static org.firstinspires.ftc.teamcode.RobotConstants.LIFT_HIGH_BUCKET;
import static org.firstinspires.ftc.teamcode.RobotConstants.LIFT_SPEC;
import static org.firstinspires.ftc.teamcode.RobotConstants.OUTTAKE_FLAT_L;
import static org.firstinspires.ftc.teamcode.RobotConstants.OUTTAKE_FLAT_R;
import static org.firstinspires.ftc.teamcode.RobotConstants.OUTTAKE_SCORE_BUCKET_L;
import static org.firstinspires.ftc.teamcode.RobotConstants.OUTTAKE_SCORE_BUCKET_R;
import static org.firstinspires.ftc.teamcode.RobotConstants.OUTTAKE_SCORE_SPEC_2_L;
import static org.firstinspires.ftc.teamcode.RobotConstants.OUTTAKE_SCORE_SPEC_2_R;
import static org.firstinspires.ftc.teamcode.RobotConstants.OUTTAKE_SCORE_SPEC_L;
import static org.firstinspires.ftc.teamcode.RobotConstants.OUTTAKE_SCORE_SPEC_R;
import static org.firstinspires.ftc.teamcode.RobotConstants.OUTTAKE_SPEC_L;
import static org.firstinspires.ftc.teamcode.RobotConstants.OUTTAKE_SPEC_R;
import static org.firstinspires.ftc.teamcode.RobotConstants.RIGHT_DROPDOWN_MAX;
import static org.firstinspires.ftc.teamcode.RobotConstants.RIGHT_DROPDOWN_MIN;
import static org.firstinspires.ftc.teamcode.RobotConstants.RIGHT_EXT_MAX;
import static org.firstinspires.ftc.teamcode.RobotConstants.RIGHT_EXT_MIN;
import static org.firstinspires.ftc.teamcode.RobotConstants.kD;
import static org.firstinspires.ftc.teamcode.RobotConstants.kF;
import static org.firstinspires.ftc.teamcode.RobotConstants.kI;
import static org.firstinspires.ftc.teamcode.RobotConstants.kP;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDFController;
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
import com.qualcomm.robotcore.hardware.TouchSensor;

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
    public int liftTarget;
    public PIDFController controller;

    // ----------------- Servos -----------------
    public Servo extL, extR, dropdownL, dropdownR,
            hangL, hangR, claw, hangPivot, outtakeFlipL, outtakeFlipR;


    // ----------------- Odometry -----------------
    public Follower follower;
    public DcMotorEx leftEnc, rightEnc, strafeEnc;

    // ----------------- Sensors -----------------
    public AnalogInput extEncL, extEncR, dropdownEncL, dropdownEncR,
            clawEnc, hangPivotEnc, outtakeFlipEnc;

    public double extLPos, extRPos, dropdownLPos, dropdownRPos,
            clawPos, hangPivotPos, outtakeFlipPos;

    public ColorSensor intakeWheel, intakeWall;
    public double intakeWallDist, intakeWheelDist;

    public DistanceSensor leftDist, rightDist;
    public double leftDistVal, rightDistVal;

    public TouchSensor verticalLimit;


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
        outtakeFlipL = hwMap.get(Servo.class, "outtakeFlipL");
        outtakeFlipR = hwMap.get(Servo.class, "outtakeFlipR");
        claw = hwMap.get(Servo.class, "claw");
        hangPivot = hwMap.get(Servo.class, "hangPivot");
        hangL = hwMap.get(Servo.class, "hangLockL");

        // ----------------- Sensors -----------------
        extEncL = hwMap.get(AnalogInput.class, "extEncL");
        extEncR = hwMap.get(AnalogInput.class, "extEncR");
        dropdownEncL = hwMap.get(AnalogInput.class, "dropdownEncL");
        dropdownEncR = hwMap.get(AnalogInput.class, "dropdownEncR");
        outtakeFlipEnc = hwMap.get(AnalogInput.class, "outtakeFlipEnc");

        intakeWheel = hwMap.get(ColorSensor.class, "intakeWheel");
        intakeWall = hwMap.get(ColorSensor.class, "intakeWall");

        verticalLimit = hwMap.get(TouchSensor.class, "verticalLimit");

        // Set ZeroPowerBehavior for Motors
        setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);

        // Set Modes for Motors
        setMotorsMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Set BulkCachingMode
        setBulkCachingMode();

        setDriveMotorDirections();
        intake.setDirection(DcMotorSimple.Direction.REVERSE);
        liftL.setDirection(DcMotorSimple.Direction.REVERSE);

        controller = new PIDFController(kP, kI, kD, kF);
        liftR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }

    // ----------------- Drivetrain -----------------
    // calculateMotorPowers(axial, lateral, yaw)
    // returns double[] (motor powers for mecanum)
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
    // sets motor power based off of motor powers
    public void setMotorPowers(double v, double v1, double v2, double v3, double speedMultiplier) {
        leftFrontDrive.setPower(v * speedMultiplier);
        leftBackDrive.setPower(v1 * speedMultiplier);
        rightFrontDrive.setPower(v2 * speedMultiplier);
        rightBackDrive.setPower(v3 * speedMultiplier);
    }

    // setMotorsMode(mode)
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

    // setBulkCachingMode()
    // Sets BulkCachingMode for all hubs
    private void setBulkCachingMode () {
        List<LynxModule> allHubs = hwMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
    }

    // setDriveMotorDirections()
    // Sets the directions for the drive motors
    private void setDriveMotorDirections() {
        leftFrontDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    // ----------------- Sensors -----------------
    // encoderUpdate()
    // Updates the encoder values
    public void encoderUpdate() {
        extLPos = extEncL.getVoltage() / 3.3 * 360;
        extRPos = extEncR.getVoltage() / 3.3 * 360;
        outtakeFlipPos = outtakeFlipEnc.getVoltage() / 3.3 * 360;
    }

    // distSensorUpdate()
    // Updates the distance sensor values
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

    // ----------------- Intake -----------------
    // pivot_down()
    // Pivots the intake down
    public void pivot_down() {
        dropdownL.setPosition(LEFT_DROPDOWN_MAX);
        dropdownR.setPosition(RIGHT_DROPDOWN_MAX);
    }

    // pivot_up()
    // Pivots the intake up
    public void pivot_up() {
        dropdownL.setPosition(LEFT_DROPDOWN_MIN);
        dropdownR.setPosition(RIGHT_DROPDOWN_MIN);
    }

    // extend(distance)
    // distance - int (1-5)
    // Extends the horizontal slides to a certain distance
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

    // retract()
    // Retracts the horizontal slides
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

    public boolean intakeWallDetect() {
        return intakeWallDist < 30;
    }

    // ----------------- Lift -----------------

    public void updateLift () {
        double power = controller.calculate(liftR.getCurrentPosition(), liftTarget);

        liftL.setPower(power);
        liftR.setPower(power);
    }

    public void extend_high_bucket() {
        liftTarget = LIFT_HIGH_BUCKET;
    }

    public void extend_high_spec() {
        liftTarget = LIFT_SPEC;
    }

    public void retractLift() {
        liftTarget = 0;
    }

    public void resetLift() {
        if (verticalLimit.isPressed()) {
            liftR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            liftTarget = 0;
        } else {
            liftTarget -= 20;
        }
    }

    public void outtake_flat() {
        outtakeFlipL.setPosition(OUTTAKE_FLAT_L);
        outtakeFlipR.setPosition(OUTTAKE_FLAT_R);
    }

    public void outtake_score_bucket() {
        outtakeFlipL.setPosition(OUTTAKE_SCORE_BUCKET_L);
        outtakeFlipR.setPosition(OUTTAKE_SCORE_BUCKET_R);
    }

    public void outtake_spec() {
        outtakeFlipL.setPosition(OUTTAKE_SPEC_L);
        outtakeFlipR.setPosition(OUTTAKE_SPEC_R);
    }

    public void outtake_score_spec() {
        outtakeFlipL.setPosition(OUTTAKE_SCORE_SPEC_L);
        outtakeFlipR.setPosition(OUTTAKE_SCORE_SPEC_R);
    }

    public void outtake_score_spec_2() {
        outtakeFlipL.setPosition(OUTTAKE_SCORE_SPEC_2_L);
        outtakeFlipR.setPosition(OUTTAKE_SCORE_SPEC_2_R);
    }

    public void claw_open() {
        claw.setPosition(CLAW_MAX);
    }

    public void claw_close() {
        claw.setPosition(CLAW_MIN);
    }


}
