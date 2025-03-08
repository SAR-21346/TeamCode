package org.firstinspires.ftc.teamcode.pedroPathing.constants;

import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.localization.Localizers;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class FConstants {
    static {
        FollowerConstants.localizers = Localizers.THREE_WHEEL;

        FollowerConstants.leftFrontMotorName = "frontLeft";
        FollowerConstants.leftRearMotorName = "backLeft";
        FollowerConstants.rightFrontMotorName = "frontRight";
        FollowerConstants.rightRearMotorName = "backRight";

        FollowerConstants.leftFrontMotorDirection = DcMotorSimple.Direction.REVERSE;
        FollowerConstants.leftRearMotorDirection = DcMotorSimple.Direction.REVERSE;
        FollowerConstants.rightFrontMotorDirection = DcMotorSimple.Direction.FORWARD;
        FollowerConstants.rightRearMotorDirection = DcMotorSimple.Direction.FORWARD;

        FollowerConstants.mass = 14.3;

        FollowerConstants.xMovement = 62.3295;
        FollowerConstants.yMovement = 43.236;

        FollowerConstants.forwardZeroPowerAcceleration = -48.4303;
        FollowerConstants.lateralZeroPowerAcceleration = -97.5251;

        FollowerConstants.translationalPIDFCoefficients.setCoefficients(0.15,0,0.0125,0);
        FollowerConstants.useSecondaryTranslationalPID = true;
        FollowerConstants.secondaryTranslationalPIDFCoefficients.setCoefficients(0.31,0,0.033,0); // Not being used, @see useSecondaryTranslationalPID

        FollowerConstants.headingPIDFCoefficients.setCoefficients(1,0,0.08,0);
        FollowerConstants.useSecondaryHeadingPID = true;
        FollowerConstants.secondaryHeadingPIDFCoefficients.setCoefficients(3,0,0.25,0); // Not being used, @see useSecondaryHeadingPID

        FollowerConstants.drivePIDFCoefficients.setCoefficients(0.013,0,0.0005,0.6,0);
        FollowerConstants.useSecondaryDrivePID = true;
        FollowerConstants.secondaryDrivePIDFCoefficients.setCoefficients(0.011,0,0.0003,0.6,0); // Not being used, @see useSecondaryDrivePID
        FollowerConstants.drivePIDFFeedForward = 0.049;

        FollowerConstants.zeroPowerAccelerationMultiplier = 6;
        FollowerConstants.centripetalScaling = 0.03;

        FollowerConstants.pathEndTimeoutConstraint = 100;
        FollowerConstants.pathEndTValueConstraint = 0.99;
        FollowerConstants.pathEndVelocityConstraint = 0.1;
        FollowerConstants.pathEndTranslationalConstraint = 0.1;
        FollowerConstants.pathEndHeadingConstraint = 0.007;

        FollowerConstants.useVoltageCompensationInAuto = true;
        FollowerConstants.nominalVoltage = 12.8;
    }
}
