package org.firstinspires.ftc.teamcode.pedroPathing.constants;

import com.pedropathing.localization.*;
import com.pedropathing.localization.constants.*;

public class LConstants {
    static {
        ThreeWheelConstants.forwardTicksToInches = 0.002938858;
        ThreeWheelConstants.strafeTicksToInches = -0.002924668;
        ThreeWheelConstants.turnTicksToInches = 0.002924668;
        ThreeWheelConstants.leftY = 6.625;
        ThreeWheelConstants.rightY = -6.625;
        ThreeWheelConstants.strafeX = -6;
        ThreeWheelConstants.leftEncoder_HardwareMapName = "backLeft";
        ThreeWheelConstants.rightEncoder_HardwareMapName = "parR";
        ThreeWheelConstants.strafeEncoder_HardwareMapName = "backRight";
        ThreeWheelConstants.leftEncoderDirection = Encoder.REVERSE;
        ThreeWheelConstants.rightEncoderDirection = Encoder.FORWARD;
        ThreeWheelConstants.strafeEncoderDirection = Encoder.REVERSE;
    }
}




