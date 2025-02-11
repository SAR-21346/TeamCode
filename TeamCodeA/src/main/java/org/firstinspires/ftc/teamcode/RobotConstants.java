package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;

@Config
public class RobotConstants {
    // Robot Lengths
    public static final double ROBOT_BACK_LENGTH = 16.25;
    public static final double ROBOT_SIDE_LENGTH = 16;

    // Intake States
    public enum IntakeState {
        INIT,
        DISTANCE_CHECK_ENABLE,
        DISTANCE_CHECK,
        EXTEND,
        PIVOT_DOWN,
        PIVOT_DOWN_BYPASS,
        INTAKE_ENABLE,
        INTAKE_ACCEPT,
        INTAKE_REJECT,
        PIVOT_UP,
        STOP
    }

    // Servo Positions
    public static double LEFT_EXT_MIN = 0.1;
    public static double LEFT_EXT_MAX = 0.62;
    public static double RIGHT_EXT_MIN = 0.1;
    public static double RIGHT_EXT_MAX = 0.62;
    public static double LEFT_DROPDOWN_MIN = 0.1;
    public static double LEFT_DROPDOWN_MAX = 0.52;
    public static double RIGHT_DROPDOWN_MIN = 0.13;
    public static double RIGHT_DROPDOWN_MAX = 0.54;

    // Intake Power
    public static double INTAKE_POWER_POS = 0.8;
    public static double INTAKE_POWER_NEG = -0.8;
}
