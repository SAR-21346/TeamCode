package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.RobotConstants.BUCKET_FLAT;
import static org.firstinspires.ftc.teamcode.RobotConstants.BUCKET_TIP;
import static org.firstinspires.ftc.teamcode.RobotConstants.EXTENSION_IN;
import static org.firstinspires.ftc.teamcode.RobotConstants.EXTENSION_MID;
import static org.firstinspires.ftc.teamcode.RobotConstants.EXTENSION_OUT;
import static org.firstinspires.ftc.teamcode.RobotConstants.INTAKE_BACKWARD;
import static org.firstinspires.ftc.teamcode.RobotConstants.INTAKE_FORWARD;
import static org.firstinspires.ftc.teamcode.RobotConstants.INTAKE_OFF;
import static org.firstinspires.ftc.teamcode.RobotConstants.PIVOT_IN;
import static org.firstinspires.ftc.teamcode.RobotConstants.PIVOT_MID;
import static org.firstinspires.ftc.teamcode.RobotConstants.PIVOT_OUT;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.Arrays;
import java.util.List;

@Config
public class MecanumTrain{
    HardwareMap hwMap; // saves HardwareMap reference to hwMap

    // ----------------- Drive Motors -----------------
    public DcMotorEx leftFrontDrive, leftBackDrive, rightFrontDrive, rightBackDrive;
    private final List<DcMotorEx> motors;

    public DcMotorEx leftEnc, rightEnc, strafeEnc;

    // ----------------- Auxillary Motors -----------------
    public DcMotorEx verticalExtension;
    public int liftStart;

    // ----------------- Servos -----------------
    public Servo intakePivot1, intakePivot2, bucket, horizontalExtension;
    public CRServo intakeServo;

    // ----------------- Camera -----------------
    public VisionPortal visionPortal;
    private WebcamName webcam1, webcam2;
    private AprilTagProcessor apriltag;

    // ----------------- Odometry -----------------
    public Follower follower;

    // ----------------- Sensors -----------------
    public ColorSensor intakeColor, bucketDetector; // TODO: Add bucket detector
    public TouchSensor verticalLimit, horizontalLimit;
    public DistanceSensor leftFrontDist, rightFrontDist;



    public MecanumTrain(HardwareMap hwMapX, ElapsedTime runtime) {
        hwMap = hwMapX; // saves reference to hwMap

        follower = new Follower(hwMap);

        // ----------------- Drive Motors -----------------
        leftFrontDrive = hwMap.get(DcMotorEx.class, "frontLeftDrive");
        rightFrontDrive = hwMap.get(DcMotorEx.class, "frontRightDrive");
        leftBackDrive = hwMap.get(DcMotorEx.class, "backLeftDrive");
        rightBackDrive = hwMap.get(DcMotorEx.class, "backRightDrive");

        motors = Arrays.asList(leftFrontDrive, leftBackDrive, rightFrontDrive, rightBackDrive);

        // ----------------- Auxillary Motors -----------------
        verticalExtension = hwMap.get(DcMotorEx.class, "verticalExt");

        leftEnc = hwMap.get(DcMotorEx.class, "parL");
        rightEnc = hwMap.get(DcMotorEx.class, "parR");
        strafeEnc = hwMap.get(DcMotorEx.class, "backLeftDrive");
        leftEnc.setDirection(DcMotorSimple.Direction.REVERSE);
        rightEnc.setDirection(DcMotorSimple.Direction.FORWARD);
        strafeEnc.setDirection(DcMotorSimple.Direction.FORWARD);

        // ----------------- Servos -----------------
        intakePivot1 = hwMap.get(Servo.class, "intakePivotL");
        intakePivot2 = hwMap.get(Servo.class, "intakePivotR");
        intakeServo = hwMap.get(CRServo.class, "intakeServo");
        bucket = hwMap.get(Servo.class, "bucket");
        horizontalExtension = hwMap.get(Servo.class, "horizontalExt");

        // ----------------- Sensors -----------------
        intakeColor = hwMap.get(ColorSensor.class, "intakeColor");
        bucketDetector = hwMap.get(ColorSensor.class, "bucketDetector");
        verticalLimit = hwMap.get(TouchSensor.class, "verticalLimit");
        rightFrontDist = hwMap.get(DistanceSensor.class, "rightFrontDist");
        leftFrontDist = hwMap.get(DistanceSensor.class, "leftFrontDist");

        // Set Modes for Motors
        setMotorsMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        verticalExtension.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Set directions for Motors
        verticalExtension.setDirection(DcMotorSimple.Direction.FORWARD);

        // Set ZeroPowerBehavior for Motors
        setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        verticalExtension.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Set BulkCachingMode for all hubs - gets sensor reads faster
        List<LynxModule> allHubs = hwMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        liftStart = verticalExtension.getCurrentPosition();

        leftFrontDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotorSimple.Direction.FORWARD);

        // TODO: Instantiate PID Controllers
        // pidLift = new PIDController(p, i, d);
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
        leftBackDrive.setPower(v1 * speedMultiplier);
        leftFrontDrive.setPower(v * speedMultiplier); // TODO: if doesn't work, change back to negative
        rightBackDrive.setPower(v3 * speedMultiplier);
        rightFrontDrive.setPower(v2 * speedMultiplier);
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

    /*
     runIntake(dir)
     dir - forward, off, backward
     passed in as a string
     */
    public void setIntakeServo(String dir) {
        if (dir.equals("forward")) {
            intakeServo.setPower(INTAKE_FORWARD);
        } else if (dir.equals("off")) {
            intakeServo.setPower(INTAKE_OFF);
        } else if (dir.equals("backward")) {
            intakeServo.setPower(INTAKE_BACKWARD);
        }
    }

    public void setHorizontalExtension(String dir) {
        if (dir.equals("in")) {
            horizontalExtension.setPosition(EXTENSION_IN);
        } else if (dir.equals("mid")) {
            horizontalExtension.setPosition(EXTENSION_MID);
        } else if (dir.equals("out")) {
            horizontalExtension.setPosition(EXTENSION_OUT);
        }
    }

    public void setIntakePivot(@NonNull String dir) {
        switch (dir) {
            case "in":
                intakePivot1.setPosition(PIVOT_IN);
                intakePivot2.setPosition(PIVOT_IN);
                break;
            case "out":
                intakePivot1.setPosition(PIVOT_OUT);
                intakePivot2.setPosition(PIVOT_OUT);
                break;
            case "mid":
                intakePivot1.setPosition(PIVOT_MID);
                intakePivot2.setPosition(PIVOT_MID);
                break;
        }
    }

    public void setBucket(String dir) {
        if (dir.equals("flat")) {
            bucket.setPosition(BUCKET_FLAT);
        } else if (dir.equals("tip")) {
            bucket.setPosition(BUCKET_TIP);
        }
    }

    public void liftExtend_lowBucket() {
        verticalExtension.setTargetPosition(1400);//TODO: change to the accurate value
        verticalExtension.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        verticalExtension.setPower(0.7); //TODO: Change to 1 once we have the correct values
    }

    public void liftExtend_highBucket() {
        verticalExtension.setTargetPosition(3970); //TODO: change to the accurate value;
        verticalExtension.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        verticalExtension.setPower(1); //TODO: Change to 1 once we have the correct values
    }
    public void liftRetract() {
        verticalExtension.setTargetPosition(liftStart);
        verticalExtension.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        verticalExtension.setPower(1); //TODO: Change to 1 once we have the correct values
    }
    public void runLift(int pos) {
        verticalExtension.setTargetPosition(pos);
        verticalExtension.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        verticalExtension.setPower(0.4); //TODO: Change to 1 once we have the correct values
    }

    public void resetLift() {
        if (verticalLimit.isPressed()) {
            verticalExtension.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        } else {
            verticalExtension.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            verticalExtension.setPower(-0.30);
        }
    }

    public void stopLift() {
        verticalExtension.setPower(0);
    }

    public boolean sampleDetected() {
        if (intakeColor instanceof DistanceSensor) {
            ColorSensor color = intakeColor;
            double distance = ((DistanceSensor) color).getDistance(DistanceUnit.MM);
            return distance < 20;
        }
        return false;
    }
  
    public boolean sampleInOuttake() {
        if (bucketDetector instanceof DistanceSensor) {
            ColorSensor color = bucketDetector;
            double distance = ((DistanceSensor) color).getDistance(DistanceUnit.MM);
            return distance < 150;
        }
    }


    public String colorDetection() {
        String color = "";
        int r = intakeColor.red(), g = intakeColor.green(), b = intakeColor.blue();
        int maxValue = Math.max(r, Math.max(g, b));
        if (maxValue == r) {
            color = "red";
        } else if (maxValue == b) {
            color = "blue";
        } else {
            color = "yellow";
        }

        return color;
    }

}
