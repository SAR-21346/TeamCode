package org.firstinspires.ftc.teamcode;


public class RobotConstants {
    // Motor Constants
    public static final double GEAR_RATIO = 0;
    public static final double WHEEL_RADIUS = 1.89;
    public static final double COUNTS_PER_REV = 537.6;
    public static final double COUNTS_PER_INCH = (COUNTS_PER_REV * GEAR_RATIO) /
            (2 * WHEEL_RADIUS * Math.PI);
    public static final double MOTOR_RPM = 300;
    
    // PID Constants
    public static double p_lift = 0, i_lift = 0, d_lift = 0, f_lift = 0;

    // Servo Positions
    public static double INTAKE_OFF = 0.5;
    public static double INTAKE_FORWARD = 1;
    public static double INTAKE_BACKWARD = 0;

    public static double EXTENSION_IN = 1;
    public static double EXTENSION_OUT = 0;

    public static double PIVOT_IN = 0;
    public static double PIVOT_OUT = 1;
}
