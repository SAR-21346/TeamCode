package org.firstinspires.ftc.teamcode.testing;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.MecanumTrain;

@Config
@TeleOp(name = "servoValueFinder", group = "testing")
public class servoValueFinder extends OpMode {
    MecanumTrain bot;
    public static double servoPos = 0;
    public static String selectedServo = "extR";

    @Override
    public void init() {
        bot = new MecanumTrain(hardwareMap);
    }

    @Override
    public void loop() {
        switch (selectedServo) {
            case "extR":
                bot.extR.setPosition(servoPos);
                break;
            case "extL":
                bot.extL.setPosition(servoPos);
                break;
            case "dropdownL":
                bot.dropdownL.setPosition(servoPos);
                break;
            case "dropdownR":
                bot.dropdownR.setPosition(servoPos);
                break;
            case "claw":
                bot.claw.setPosition(servoPos);
                break;
            case "hangPivot":
                bot.hangPivot.setPosition(servoPos);
                break;
            case "hangL":
                bot.hangL.setPosition(servoPos);
                break;
            case "hangR":
                bot.hangR.setPosition(servoPos);
                break;
            case "outtakeFlipL":
                bot.outtakeFlipL.setPosition(servoPos);
                break;
            case "outtakeFlipR":
                bot.outtakeFlipR.setPosition(servoPos);
                break;
        }

        telemetry.addData("selected servo", selectedServo);
        telemetry.addData("servoPos", servoPos);
        telemetry.update();
    }
}
