package org.firstinspires.ftc.teamcode.pedroPathing.constants;

import com.pedropathing.localization.Encoder;
import com.pedropathing.localization.constants.ThreeWheelConstants;

public class LConstants {
    static {
        ThreeWheelConstants.forwardTicksToInches = 0.0029631347485395304;
        ThreeWheelConstants.strafeTicksToInches = 0.002938289528850785;
        ThreeWheelConstants.turnTicksToInches = 0.0029684557463382547; //
        ThreeWheelConstants.leftY = 8.0625;
        ThreeWheelConstants.rightY = -8.0625;
        ThreeWheelConstants.strafeX = -2.625;
        ThreeWheelConstants.leftEncoder_HardwareMapName = "parL";
        ThreeWheelConstants.rightEncoder_HardwareMapName = "parR";
        ThreeWheelConstants.strafeEncoder_HardwareMapName = "backLeftDrive";
        ThreeWheelConstants.leftEncoderDirection = Encoder.REVERSE;
        ThreeWheelConstants.rightEncoderDirection = Encoder.FORWARD;
        ThreeWheelConstants.strafeEncoderDirection = Encoder.FORWARD;
    }
}




