package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;

@Config
public class RobotConstants {
    // PIDF Coefficients
    public static double kP = 0.014;
    public static double kI = 0.0;
    public static double kD = 0.0;
    public static double kF = 0.0;


    // Robot Lengths
    public static final double ROBOT_BACK_LENGTH = 16.25;
    public static final double ROBOT_SIDE_LENGTH = 16;
    public static final double MAX_EXT_DIST= 33;
    public static final double ROBOT_CENTER_TO_MAX_EXT = MAX_EXT_DIST-(ROBOT_SIDE_LENGTH/2);

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

    public enum OuttakeState {
        INIT,
        START,
        INTAKE_GRAB,
        EXTEND_HIGH_BUCKET,
        SCORE_HIGH_BUCKET,
        SPEC_PICKUP,
        EXTEND_HIGH_SPEC,
        SCORE_HIGH_SPEC,
        RETRACT,
        STOP
    }

    // Servo Positions
    // INTAKE
    public static double LEFT_EXT_MIN = 0;
    public static double LEFT_EXT_MAX = 0.62;
    public static double RIGHT_EXT_MIN = 0;
    public static double RIGHT_EXT_MAX = 0.62;
    public static double LEFT_DROPDOWN_MIN = 0.1;
    public static double LEFT_DROPDOWN_MAX = 0.52;
    public static double RIGHT_DROPDOWN_MIN = 0.13;
    public static double RIGHT_DROPDOWN_MAX = 0.54;

    // OUTTAKE
    public static double OUTTAKE_FLAT_L = 0;
    public static double OUTTAKE_FLAT_R = 0;
    public static double OUTTAKE_SCORE_BUCKET_L = 0.7;
    public static double OUTTAKE_SCORE_BUCKET_R = 0.7;
    public static double OUTTAKE_SPEC_L = 0.9;
    public static double OUTTAKE_SPEC_R = 0.9;
    public static double OUTTAKE_SCORE_SPEC_L = 0.3;
    public static double OUTTAKE_SCORE_SPEC_R = 0.3;
    public static double OUTTAKE_SCORE_SPEC_2_L = 0.5;
    public static double OUTTAKE_SCORE_SPEC_2_R = 0.5;

    // CLAW
    public static double CLAW_MIN = 0;
    public static double CLAW_MAX = 0.6;

    // Intake Power
    public static double INTAKE_POWER_POS = 0.8;
    public static double INTAKE_POWER_NEG = -0.8;

    // Lift Positions
    public static int LIFT_MIN = 0;
    public static int LIFT_HIGH_BUCKET = 3350;
    public static int LIFT_SPEC = 1800;
    public static int LIFT_MAX = 3400;
}
