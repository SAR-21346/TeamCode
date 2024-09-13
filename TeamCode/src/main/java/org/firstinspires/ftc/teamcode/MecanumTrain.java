package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.Arrays;
import java.util.List;

@Config
public class MecanumTrain{
    // Drive Motors
    public DcMotorEx leftFrontDrive;
    public DcMotorEx leftBackDrive;
    public DcMotorEx rightFrontDrive;
    public DcMotorEx rightBackDrive;

      // TODO: Add extra motor definitions based on game
//    // Auxiliary Motors
//    public DcMotorEx leftSlide;
//    public DcMotorEx rightSlide;
//    public DcMotorEx spinTake;
//    public DcMotorEx outMotor;
//
//    // Servos
//    public Servo claw;
//    public Servo drone;

    // TODO: Add Odometry, Vision, and other sensors

//    // Camera
//    public VisionPortal visionPortal;
//    public BluePropPipeline pipeline;
//    public RedPropPipeline pipelineRed;

    HardwareMap hwMap = null; // saves HardwareMap reference to hwMap

    // TODO: Configure motor constants for both types of motors we use
    // Motor Constants


    // REV Motor Definitions
/*
    static final double COUNTS_PER_REV = 537.6;
    static final double REV_COUNTS_PER_INCH = (REV_COUNTS_PER_REV * GEAR_RATIO) /
            (2 * WHEEL_RADIUS * Math.PI);

     GoBilda Motor Definitions
    static final double COUNTS_PER_REV = 537.6;

//    static final double COUNTS_PER_INCH = (COUNTS_PER_REV * GEAR_RATIO) /
//            (2 * WHEEL_RADIUS * Math.PI);
    public static int target = 0;

//    public static int target = 0;

      // TODO: PID Controller definitions
//    private PIDController controllerArm;
//    private PIDController controllerLift;
//    public static double p = 0, i = 0, d = 0, f = 0;
*/
    private List<DcMotorEx> motors;
/*
    // TODO: Delete these constants if not used
    // Initial Motor Positions
    public double arm_start;
    public int liftL_start;
    public int liftR_start;

    // Claw and Drone Position Constants
    public static double CLAW_OPEN = 0.20;
    public static double CLAW_CLOSED = 0;

    public static double DRONE_OPEN = 0;
    public static double DRONE_CLOSED = .7;

    // Lift Speeds
    public static int liftL_speed = 450;
    public static int liftR_speed = 450;

    public Gamepad.RumbleEffect rumbleEffect;
*/
    public MecanumTrain(HardwareMap hwMapX, ElapsedTime runtime) {
        hwMap = hwMapX; // saves reference to hwMap

        // TODO: Add Odometry
        // odometry = new SampleMecanumDrive(hwMap);


        // Drive Motors
        leftFrontDrive = hwMap.get(DcMotorEx.class, "FLdrive");
        leftBackDrive = hwMap.get(DcMotorEx.class, "BLdrive");
        rightFrontDrive = hwMap.get(DcMotorEx.class, "FRdrive");
        rightBackDrive = hwMap.get(DcMotorEx.class, "BRdrive");
/*
        // TODO: Configure Auxiliary Motors
        leftSlide = hwMap.get(DcMotorEx.class, "Lslide");
        rightSlide = hwMap.get(DcMotorEx.class, "Rslide");
        spinTake = hwMap.get(DcMotorEx.class, "Spintake");
        outMotor = hwMap.get(DcMotorEx.class, "Outtake");
*/

        // TODO: Add Servos
//        claw = hwMap.get(Servo.class, "claw");
//        drone = hwMap.get(Servo.class, "drone");

        // Motor List
        motors = Arrays.asList(leftFrontDrive, leftBackDrive, rightFrontDrive, rightBackDrive);

        // Set Modes for Motors
        setMotorsMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        // outMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // Set directions for Motors
        leftBackDrive.setDirection(DcMotorSimple.Direction.REVERSE);
//        leftSlide.setDirection(DcMotorSimple.Direction.REVERSE);

        // Set ZeroPowerBehavior for Motors
        setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE); /*
        leftSlide.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rightSlide.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        outMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        leftSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
*/
        // TODO: Instantiate PID Controllers
        // controllerArm = new PIDController(p, i, d);

        // Gamepad Rumble Effect
//        rumbleEffect = new Gamepad.RumbleEffect.Builder()
//                .addStep(0.5, 0.5, 500)
//                .addStep(0.0,0.0, 300)
//                .addStep(1.0, 1.0, 500)
//                .addStep(0.0, 0.0, 1000)
//                .build();
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
     runIntake(power)
     power - double (power for spinTake)
     */
     // public void runIntake(double power) { spinTake.setPower(power); }

    // runLift(pos)
    // pos - int (target position for leftSlide and rightSlide)
    /*
    public void runLift (int pos) {
        leftSlide.setTargetPosition((pos + liftL_start)); // liftL_start is the initial position of the left slide
        leftSlide.setVelocity(liftL_speed); // Was told to change speeds for the individual motors
        leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION); // Allows motor to move via encoder
        rightSlide.setTargetPosition(pos + liftR_start); // liftR_start is the initial position of the right slide
        rightSlide.setVelocity(liftR_speed); // Was told to change speeds for the individual motors
        rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION); // Allows motor to move via encoder
    }


    public void openClaw() { claw.setPosition(CLAW_OPEN); }
    public void closeClaw() { claw.setPosition(CLAW_CLOSED); }
    public void openDrone() { drone.setPosition(DRONE_OPEN); }
    public void closeDrone() { drone.setPosition(DRONE_CLOSED); }

    */
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
    /*
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

    public void initEocvRed(HardwareMap hwMap) {
        pipelineRed = new RedPropPipeline();
        VisionPortal.Builder builder = new VisionPortal.Builder();
        builder.setCamera(hwMap.get(WebcamName.class, "camera"));
        builder.setCameraResolution(new Size(800, 448));
        builder.addProcessor(pipelineRed);
        visionPortal = builder.build();
    }

    public void initEocvBlue(HardwareMap hwMap) {
        pipeline = new BluePropPipeline();
        VisionPortal.Builder builder = new VisionPortal.Builder();
        builder.setCamera(hwMap.get(WebcamName.class, "camera"));
        builder.setCameraResolution(new Size(800, 448));
        builder.addProcessor(pipeline);
        visionPortal = builder.build();
    }
    */

}
