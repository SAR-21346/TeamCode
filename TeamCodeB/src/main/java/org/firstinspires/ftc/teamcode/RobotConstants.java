package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;

@Config
public class RobotConstants {
    // Motor Constants
    public static final double GEAR_RATIO = 12;
    public static final double WHEEL_RADIUS = 1.89;
    public static final double COUNTS_PER_REV = 537.6;
    public static final double COUNTS_PER_INCH = (COUNTS_PER_REV * GEAR_RATIO) /
            (2 * WHEEL_RADIUS * Math.PI);
    public static final double MOTOR_RPM = 300;

    // GoBilda Odometry Constants
    public static final double WHEEL_RADIUS_ODO = 0.94;
    public static final double COUNTS_PER_REV_ODO = 2000;

    // Robot Lengths
    public static final double ROBOT_BACK_LENGTH = 17;
    public static final double ROBOT_SIDE_LENGTH = 17;


    // Intake States
    public enum IntakeState {
        INTAKE_INIT,
        INTAKE_START,
        INTAKE_EXTEND,
        INTAKE_FLIP_IN,
        INTAKE_FLIP_OUT,
        INTAKE_SPIN,
        INTAKE_STOP_SPIN,
        INTAKE_REJECT,
        INTAKE_SAMPLE_IN,
        INTAKE_RETRACT,
        INTAKE_RELEASE,
        INTAKE_STOP;
    }

    public enum LiftState {
        LIFT_INIT,
        LIFT_START,
        LIFT_EXTEND_LOW,
        LIFT_EXTEND_HIGH,
        BUCKET_TIP,
        LIFT_RETRACT,
        LIFT_STOP,
        LIFT_RELEASE;
    }

    // Servo Positions
    public static final double INTAKE_OFF = 0;
    public static final double INTAKE_FORWARD = 1;
    public static final double INTAKE_BACKWARD = -1;

    public static final double EXTENSION_IN = 0;
    public static final double EXTENSION_MID = 0.26;
    public static final double EXTENSION_OUT = 0.5;

    public static final double PIVOT_IN = 0.05;
    public static final double PIVOT_MID = 0.5;
    public static final double PIVOT_OUT = 1;

    public static double BUCKET_FLAT = 0.6; //0.6
    public static double BUCKET_TIP = 0.25; //0
}
