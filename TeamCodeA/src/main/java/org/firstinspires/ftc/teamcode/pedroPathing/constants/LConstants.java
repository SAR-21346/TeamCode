package org.firstinspires.ftc.teamcode.pedroPathing.constants;

import com.pedropathing.localization.*;
import com.pedropathing.localization.constants.*;

public class LConstants {
    static {
        ThreeWheelConstants.forwardTicksToInches = 0.00294567; // 0.0029468431, 0.00294177178, 0.0029454338, 0.002945901, 0.00294823653, 0.0029439639
        ThreeWheelConstants.strafeTicksToInches = 0.002982075; // 0.002971800, 0.002980075, 0.0029765267,  0.002981959,0.0029788379, 0.002955759, 0.00298381407, 0.0029733620
        ThreeWheelConstants.turnTicksToInches = 0.003003217969;
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




