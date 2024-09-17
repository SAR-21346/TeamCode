package org.firstinspires.ftc.teamcode;


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

    // PID Constants
    public static final double p_lift = 0, i_lift = 0, d_lift = 0, f_lift = 0;

    // Intake States
    public enum IntakeState {
        INTAKE_START,
        INTAKE_EXTEND,
        INTAKE_FLIP_IN,
        INTAKE_FLIP_OUT,
        INTAKE_SPIN,
        INTAKE_SAMPLE_IN,
        INTAKE_SAMPLE_OUT,
        INTAKE_RETRACT,
        INTAKE_RELEASE;
    }

    public enum OuttakeState {
        OUTTAKE_START,
        OUTTAKE_TIP,
        OUTTAKE_SAMPLE_RELEASED,
        OUTTAKE_RETRACT;
    }


    // Servo Positions
    public static final double INTAKE_OFF = 0.5;
    public static final double INTAKE_FORWARD = 1;
    public static final double INTAKE_BACKWARD = 0;

    public static final double EXTENSION_IN = 1;
    public static final double EXTENSION_OUT = 0;

    public static final double PIVOT_IN = 0;
    public static final double PIVOT_OUT = 1;
}
