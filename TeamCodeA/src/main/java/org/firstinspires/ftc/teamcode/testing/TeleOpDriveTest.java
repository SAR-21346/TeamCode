package org.firstinspires.ftc.teamcode.testing;


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

        telemetry.addData("leftFront", powers[0]);
        telemetry.addData("leftRear", powers[1]);
        telemetry.addData("rightFront", powers[2]);
        telemetry.addData("rightRear", powers[3]);

        telemetry.update();
    }
}
