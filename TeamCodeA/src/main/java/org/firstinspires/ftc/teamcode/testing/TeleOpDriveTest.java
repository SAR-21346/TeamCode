package org.firstinspires.ftc.teamcode.testing;


import static org.firstinspires.ftc.teamcode.RobotConstants.OUTTAKE_FLAT_L;
import static org.firstinspires.ftc.teamcode.RobotConstants.OUTTAKE_FLAT_R;
import static org.firstinspires.ftc.teamcode.RobotConstants.OUTTAKE_SCORE_BUCKET_L;
import static org.firstinspires.ftc.teamcode.RobotConstants.OUTTAKE_SCORE_BUCKET_R;
import static org.firstinspires.ftc.teamcode.RobotConstants.OUTTAKE_SCORE_SPEC_L;
import static org.firstinspires.ftc.teamcode.RobotConstants.OUTTAKE_SCORE_SPEC_R;
import static org.firstinspires.ftc.teamcode.RobotConstants.OUTTAKE_SPEC_L;
import static org.firstinspires.ftc.teamcode.RobotConstants.OUTTAKE_SPEC_R;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.MecanumTrain;

@TeleOp(name = "Drive Test", group = "testing")
public class TeleOpDriveTest extends OpMode {
    MecanumTrain bot;

    @Override
    public void init() {
        bot = new MecanumTrain(hardwareMap);
        bot.liftTarget = 0;

    }

    @Override
    public void loop() {
        double y = -gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x;
        double rx = gamepad1.right_stick_x;

        double[] powers = bot.calculateMotorPowers(y,x,rx);
        bot.setMotorPowers(powers[0], powers[1], powers[2], powers[3], 1);

        if (gamepad2.dpad_up) {
            bot.outtakeFlipL.setPosition(OUTTAKE_FLAT_L);
            bot.outtakeFlipR.setPosition(OUTTAKE_FLAT_R);
        }
        if (gamepad2.dpad_down) {
            bot.outtakeFlipL.setPosition(OUTTAKE_SPEC_L);
            bot.outtakeFlipR.setPosition(OUTTAKE_SPEC_R);
        }
        if (gamepad2.dpad_left) {
            bot.outtakeFlipL.setPosition(OUTTAKE_SCORE_BUCKET_L);
            bot.outtakeFlipR.setPosition(OUTTAKE_SCORE_BUCKET_R);
        }
        if (gamepad2.dpad_right) {
            bot.outtakeFlipL.setPosition(OUTTAKE_SCORE_SPEC_L);
            bot.outtakeFlipR.setPosition(OUTTAKE_SCORE_SPEC_R);
        }

        if (gamepad2.options) {
            bot.resetLift();
        }


        if (gamepad2.a) {
            bot.liftTarget = 3000;
        } else if (gamepad2.b) {
            bot.liftTarget = 1500;
        } else if (gamepad2.x) {
            bot.liftTarget = 0;
        } else if (gamepad2.y) {
            bot.liftTarget = 3400;
        }

        bot.updateLift();

        bot.encoderUpdate();
        telemetry.addData("lift", bot.liftR.getCurrentPosition());
        telemetry.addData("liftTarget", bot.liftTarget);
        telemetry.addData("leftFront", powers[0]);
        telemetry.addData("leftRear", powers[1]);
        telemetry.addData("rightFront", powers[2]);
        telemetry.addData("rightRear", powers[3]);
        telemetry.addLine();

        telemetry.addData("encL", bot.extLPos);
        telemetry.addData("encR", bot.extRPos);
        telemetry.addData("outtakeFlipEnc", bot.outtakeFlipPos);


        telemetry.addData("intakeWheel", bot.intakeWheelDetect());

        if (bot.intakeWall instanceof DistanceSensor) {
            ColorSensor color = bot.intakeWall;
            double distance = ((DistanceSensor) color).getDistance(DistanceUnit.MM);
            telemetry.addData("intakeWallDist", distance);
        }

        if (bot.intakeWheel instanceof DistanceSensor) {
            ColorSensor color = bot.intakeWheel;
            double distance = ((DistanceSensor) color).getDistance(DistanceUnit.MM);
            telemetry.addData("intakeWheelDist", distance);
        }

        telemetry.addData("verticalLimit", bot.verticalLimit.isPressed());

        telemetry.addData("leftEnc", bot.leftEnc.getCurrentPosition());
        telemetry.addData("rightEnc", bot.rightEnc.getCurrentPosition());
        telemetry.addData("strafeEnc", bot.strafeEnc.getCurrentPosition());
        telemetry.update();
    }
}
