package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.MecanumTrain;
import org.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "TeleOpMain")
public class TeleOpMain {
    private ElapsedTime runtime = new ElapsedTime();

    MecanumTrain bot;

    public void runOpMode() {
        bot = new MecanumTrain();

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
            double[] motorPowers = mecanumTrain.calculateMotorPowers(axial, lateral, yaw);

            // set motor powers
            bot.lfDriveController.setPower(motorPowers[0]);
            bot.leftBackDrive.setPower(motorPowers[1]);
            bot.rightFrontDrive.setPower(motorPowers[2]);
            bot.rightBackDrive.setPower(motorPowers[3]);

            mecanumTrain.statusToTelemetry();
        }

        // Stop all motion;
        bot.trainStop();
    }
}
