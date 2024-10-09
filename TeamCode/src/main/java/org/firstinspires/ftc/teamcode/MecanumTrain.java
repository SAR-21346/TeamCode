package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.RobotConstants.BUCKET_FLAT;
import static org.firstinspires.ftc.teamcode.RobotConstants.BUCKET_TIP;
import static org.firstinspires.ftc.teamcode.RobotConstants.EXTENSION_IN;
import static org.firstinspires.ftc.teamcode.RobotConstants.EXTENSION_OUT;
import static org.firstinspires.ftc.teamcode.RobotConstants.INTAKE_BACKWARD;
import static org.firstinspires.ftc.teamcode.RobotConstants.INTAKE_FORWARD;
import static org.firstinspires.ftc.teamcode.RobotConstants.INTAKE_OFF;
import static org.firstinspires.ftc.teamcode.RobotConstants.PIVOT_IN;
import static org.firstinspires.ftc.teamcode.RobotConstants.PIVOT_MID;
import static org.firstinspires.ftc.teamcode.RobotConstants.PIVOT_OUT;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
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

    // TODO: PID Controller definitions
    private PIDController pidLift;


    public MecanumTrain(HardwareMap hwMapX, ElapsedTime runtime) {
        hwMap = hwMapX; // saves reference to hwMap

        follower = new Follower(hwMap);

        // ----------------- Drive Motors -----------------
        leftFrontDrive = hwMap.get(DcMotorEx.class, "FLdrive");
        rightFrontDrive = hwMap.get(DcMotorEx.class, "FRdrive");
        leftBackDrive = hwMap.get(DcMotorEx.class, "BLdrive");
        rightBackDrive = hwMap.get(DcMotorEx.class, "BRdrive");

        motors = Arrays.asList(leftFrontDrive, leftBackDrive, rightFrontDrive, rightBackDrive);

        // ----------------- Auxillary Motors -----------------
        verticalExtension = hwMap.get(DcMotorEx.class, "verticalExt");

        // ----------------- Servos -----------------
        intakePivot1 = hwMap.get(Servo.class, "intakePivot");
        intakePivot2 = hwMap.get(Servo.class, "intakePivot2");
        intakeServo = hwMap.get(CRServo.class, "intakeServo");
        bucket = hwMap.get(Servo.class, "bucket");
        horizontalExtension = hwMap.get(Servo.class, "horizontalExt");

        // ----------------- Sensors -----------------
        intakeColor = hwMap.get(ColorSensor.class, "intakeColor");
//        bucketDetector = hwMap.get(ColorSensor.class, "bucketDetector");
        horizontalLimit = hwMap.get(TouchSensor.class, "horizontalLimit");
        verticalLimit = hwMap.get(TouchSensor.class, "verticalLimit");
        rightFrontDist = hwMap.get(DistanceSensor.class, "rightFrontDist");
        leftFrontDist = hwMap.get(DistanceSensor.class, "leftFrontDist");

        // Set Modes for Motors
        setMotorsMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        verticalExtension.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Set directions for Motors
        verticalExtension.setDirection(DcMotorSimple.Direction.REVERSE);

        // Set ZeroPowerBehavior for Motors
        setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        verticalExtension.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Set BulkCachingMode for all hubs - gets sensor reads faster
        List<LynxModule> allHubs = hwMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        liftStart = verticalExtension.getCurrentPosition();
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
        leftFrontDrive.setPower(-v * speedMultiplier); // TODO: if doesn't work, change back to negative
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
        } else if (dir.equals("out")) {
            horizontalExtension.setPosition(EXTENSION_OUT);
        }
    }

    public void setIntakePivot(String dir) {
        if (dir.equals("in")) {
            intakePivot1.setPosition(PIVOT_IN);
            intakePivot2.setPosition(PIVOT_IN);
        } else if (dir.equals("out")) {
            intakePivot1.setPosition(PIVOT_OUT);
            intakePivot2.setPosition(PIVOT_OUT);
        } else if (dir.equals("mid")) {
            intakePivot1.setPosition(PIVOT_MID);
            intakePivot2.setPosition(PIVOT_MID);
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
        verticalExtension.setTargetPosition(450);//TODO: change to the accurate value
        verticalExtension.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        verticalExtension.setPower(0.4); //TODO: Change to 1 once we have the correct values
    }

    public void liftExtend_highBucket() {
        verticalExtension.setTargetPosition(1300); //TODO: change to the accurate value
        verticalExtension.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        verticalExtension.setPower(0.4); //TODO: Change to 1 once we have the correct values
    }
    public void liftRetract() {
        verticalExtension.setTargetPosition(liftStart);
        verticalExtension.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        verticalExtension.setPower(0.4); //TODO: Change to 1 once we have the correct values
    }
    public void runLift(int pos) {
        verticalExtension.setTargetPosition(pos);
        verticalExtension.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        verticalExtension.setPower(0.4); //TODO: Change to 1 once we have the correct values
    }

    public void resetLift() {
        verticalExtension.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void stopLift() {
        verticalExtension.setPower(0);
    }

    public boolean sampleDetected() {
        if (intakeColor instanceof DistanceSensor) {
            ColorSensor color = intakeColor;
            double distance = ((DistanceSensor) color).getDistance(DistanceUnit.MM);
            return distance < 30;
        }
        return false;
    }


}
