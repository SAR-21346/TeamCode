package org.firstinspires.ftc.teamcode.pedroPathing.constants;

import com.pedropathing.localization.*;
import com.pedropathing.localization.constants.*;

public class LConstants {
    static {
        ThreeWheelConstants.forwardTicksToInches = 	0.002941; //  0.002926152, 0.0029458987,  0.002911096, 0.002939359, 0.0029427327, 0.00294636326, 0.0029105637, 0.0029442111
        ThreeWheelConstants.strafeTicksToInches =  -0.0029641; // -0.003102108, 0.0029608858, 0.0029150875,  0.00299381291, 0.002966502375, 0.003015066195,  0.0030183372, 0.0029657692, 0.0029913114, 0.003005978697
        ThreeWheelConstants.turnTicksToInches = 0.0029186; // 0.002917526, 0.00291964
        ThreeWheelConstants.leftY = 6.5;
        ThreeWheelConstants.rightY = -6.25;
        ThreeWheelConstants.strafeX = -7;
        ThreeWheelConstants.leftEncoder_HardwareMapName = "backLeft";
        ThreeWheelConstants.rightEncoder_HardwareMapName = "parR";
        ThreeWheelConstants.strafeEncoder_HardwareMapName = "backRight";
        ThreeWheelConstants.leftEncoderDirection = Encoder.REVERSE;
        ThreeWheelConstants.rightEncoderDirection = Encoder.FORWARD;
        ThreeWheelConstants.strafeEncoderDirection = Encoder.REVERSE;
    }
}




