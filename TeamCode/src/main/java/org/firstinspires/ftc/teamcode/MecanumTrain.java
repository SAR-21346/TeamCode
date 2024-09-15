package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.RobotConstants.EXTENSION_IN;
import static org.firstinspires.ftc.teamcode.RobotConstants.EXTENSION_OUT;
import static org.firstinspires.ftc.teamcode.RobotConstants.INTAKE_BACKWARD;
import static org.firstinspires.ftc.teamcode.RobotConstants.INTAKE_FORWARD;
import static org.firstinspires.ftc.teamcode.RobotConstants.INTAKE_OFF;
import static org.firstinspires.ftc.teamcode.RobotConstants.PIVOT_IN;
import static org.firstinspires.ftc.teamcode.RobotConstants.PIVOT_OUT;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.vision.VisionPortal;

import java.util.Arrays;
import java.util.List;

@Config
public class MecanumTrain{
    HardwareMap hwMap; // saves HardwareMap reference to hwMap

    // ----------------- Drive Motors -----------------
    public DcMotorEx leftFrontDrive;
    public DcMotorEx leftBackDrive;
    public DcMotorEx rightFrontDrive;
    public DcMotorEx rightBackDrive;

    private final List<DcMotorEx> motors;

    // ----------------- Auxillary Motors -----------------
    public DcMotorEx verticalExtension;

    // ----------------- Servos -----------------
    public Servo intakePivot;
    public CRServo intakeServo;
    public Servo horizontalExtension;

    // TODO: Add Odometry, Vision, and other sensors
    // ----------------- Camera -----------------
    public VisionPortal visionPortal;
    private WebcamName webcam1, webcam2;

    // ----------------- Odometry -----------------
    private Follower follower;

    // ----------------- Sensors -----------------
    public ColorSensor intakeColor;
    public TouchSensor verticalLimit; // change this in robot config
    public TouchSensor horizontalLimit;
//    public TouchSensor leftFrontTouch;
//    public TouchSensor rightFrontTouch;
    // TODO: PID Controller definitions
    private PIDController pidLift;


    public MecanumTrain(HardwareMap hwMapX, ElapsedTime runtime) {
        hwMap = hwMapX; // saves reference to hwMap

        // TODO: Add Odometry
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
        intakePivot = hwMap.get(Servo.class, "intakePivot");
        intakeServo = hwMap.get(CRServo.class, "intakeServo");
        horizontalExtension = hwMap.get(Servo.class, "horizontalExt");

        // ----------------- Sensors -----------------
        intakeColor = hwMap.get(ColorSensor.class, "intakeColor");
        horizontalLimit = hwMap.get(TouchSensor.class, "horizontalLimit");
        verticalLimit = hwMap.get(TouchSensor.class, "verticalLimit");
        // rightFrontTouch = hwMap.get(TouchSensor.class, "rightFrontTouch");
        // leftFrontTouch = hwMap.get(TouchSensor.class, "leftFrontTouch");

        // Set Modes for Motors
        setMotorsMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Set directions for Motors
        leftBackDrive.setDirection(DcMotorSimple.Direction.REVERSE);

        // Set ZeroPowerBehavior for Motors
        setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

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
        double multiplier = .55;
        motorPowers[0] = (axial + lateral + yaw) * multiplier;
        motorPowers[1] = (axial - lateral - yaw) * multiplier;
        motorPowers[2] = (axial - lateral + yaw) * multiplier;
        motorPowers[3] = (axial + lateral - yaw) * multiplier;
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

    // TODO: Configure runner methods
    /*
     runIntake(dir)
     dir - forward, off, backward
     passed in as a string
     */
     public void runIntake(String dir) {
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
             intakePivot.setPosition(PIVOT_IN);
         } else if (dir.equals("out")) {
             intakePivot.setPosition(PIVOT_OUT);
         }
     }

    // update()
    // Updates the telemetry with the current encoder values for the arm
    // Should be called in a loop
//    public void updateArmPID(double armPos) {
//        controllerArm.setPID(p, i, d);
//        double pid = controllerArm.calculate(armPos, target + arm_start);
//        double ff =  Math.cos(Math.toRadians((target + arm_start) / (COUNTS_PER_REV / (2 * Math.PI)))) * f;
//        double power = pid + ff;
//        outMotor.setPower(power);
//    }

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
}
