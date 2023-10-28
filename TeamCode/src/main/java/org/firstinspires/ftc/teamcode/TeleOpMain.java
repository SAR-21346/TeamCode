package org.firstinspires.ftc.teamcode;


import org.firstinspires.ftc.teamcode.MecanumTrain;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "TeleOpMain")
public class TeleOpMain extends LinearOpMode{
    private ElapsedTime runtime = new ElapsedTime();

    MecanumTrain bot;

    public void runOpMode() {
        bot = new MecanumTrain();
        bot.init(hardwareMap);
        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            // get controller inputs
            double axial = -gamepad1.left_stick_y; // Note: pushing stick forward gives negative value
            double lateral = gamepad1.left_stick_x;
            double yaw = gamepad1.right_stick_x;

            // calculate motor powers
            double[] motorPowers = bot.calculateMotorPowers(axial, lateral, yaw);

            // set motor powers
            bot.leftFrontDrive.setPower(motorPowers[0]);
            bot.leftBackDrive.setPower(motorPowers[1]);
            bot.rightFrontDrive.setPower(motorPowers[2]);
            bot.rightBackDrive.setPower(motorPowers[3]);

            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Front left/Right", "%4.2f, %4.2f", motorPowers[0], motorPowers[2]);
            telemetry.addData("Back  left/Right", "%4.2f, %4.2f", motorPowers[2], motorPowers[3]);
            telemetry.addData("Axial,Lateral,Yaw", "%4.2f, %4.2f, %4.2f", axial, lateral, yaw);
            telemetry.update();
        }

        // Stop all motion;
        bot.trainStop();
    }
}
