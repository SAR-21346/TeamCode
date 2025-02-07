package org.firstinspires.ftc.teamcode.testing;


import static org.firstinspires.ftc.teamcode.RobotConstants.LEFT_EXT_MAX;
import static org.firstinspires.ftc.teamcode.RobotConstants.LEFT_EXT_MIN;
import static org.firstinspires.ftc.teamcode.RobotConstants.RIGHT_EXT_MAX;
import static org.firstinspires.ftc.teamcode.RobotConstants.RIGHT_EXT_MIN;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.MecanumTrain;

@TeleOp(name = "Drive Test", group = "testing")
public class TeleOpDriveTest extends OpMode {
    MecanumTrain bot;

    @Override
    public void init() {
        bot = new MecanumTrain(hardwareMap);
    }

    @Override
    public void loop() {
        double y = -gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x;
        double rx = gamepad1.right_stick_x;

        double[] powers = bot.calculateMotorPowers(y,x,rx);
        bot.setMotorPowers(powers[0], powers[1], powers[2], powers[3], 1);

        if (gamepad2.dpad_up) {
            bot.extL.setPosition(LEFT_EXT_MIN);
            bot.extR.setPosition(RIGHT_EXT_MIN);
        }
        if (gamepad2.dpad_down) {
            bot.extL.setPosition(LEFT_EXT_MAX);
            bot.extR.setPosition(RIGHT_EXT_MAX);
        }

        telemetry.addData("leftFront", powers[0]);
        telemetry.addData("leftRear", powers[1]);
        telemetry.addData("rightFront", powers[2]);
        telemetry.addData("rightRear", powers[3]);
        telemetry.addLine();

        double encL = bot.extEncL.getVoltage() / 3.3 * 360;
        double encR = bot.extEncR.getVoltage() / 3.3 * 360;

        telemetry.addData("encL", encL);
        telemetry.addData("encR", encR);

        telemetry.update();
    }
}
