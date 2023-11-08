package org.firstinspires.ftc.teamcode;


import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.MecanumTrain;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "TeleOpMain")
public class TeleOpMain extends LinearOpMode{
    private final ElapsedTime runtime = new ElapsedTime();

    MecanumTrain bot;

    static final double SPEED_MULTIPLIER = 0.3;

    public void runOpMode() {
        //bot initialization
        bot = new MecanumTrain(hardwareMap);

        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();
        runtime.reset();

        double intakePower = 0.0;
        double liftPower = 0.0;

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            // get controller inputs
            double axial = -gamepad1.left_stick_y; // Note: pushing stick forward gives negative value
            double lateral = gamepad1.left_stick_x;
            double yaw = gamepad1.right_stick_x;

            if (gamepad1.dpad_down) { intakePower = -0.65; }
            if (gamepad1.dpad_up) { intakePower = 0.65; }
            if (gamepad1.dpad_left) { intakePower = 0; }

            // TODO: Test values for Lift
            if (gamepad1.triangle) {
                liftPower += 0.005;
            }
            if (gamepad1.cross) {
                liftPower -= 0.005;
            }
            if (gamepad1.square) {
                liftPower = 0.0;
            }

            bot.liftL_start = bot.leftSlide.getCurrentPosition();
            bot.liftR_start = bot.rightSlide.getCurrentPosition();

            // calculate motor powers
            double[] motorPowers = bot.calculateMotorPowers(axial, lateral, yaw);

            if (gamepad1.left_stick_button || gamepad1.right_stick_button) {
                bot.setMotorPowers(motorPowers[0] * SPEED_MULTIPLIER,
                                  motorPowers[1] * SPEED_MULTIPLIER,
                                  motorPowers[2] * SPEED_MULTIPLIER,
                                  motorPowers[3] * SPEED_MULTIPLIER);
            } else {
                bot.setMotorPowers(motorPowers[0], motorPowers[1], motorPowers[2], motorPowers[3]);
            }

            telemetry.addData("Pid info", "%4.2f", bot.updateArmPID());

            bot.runIntake(intakePower);


            // TODO: Run ArmPID test, make sure values work.
            if (gamepad1.circle) {
                bot.runOuttake(bot.updateArmPID());
                telemetry.addData("pos: ", bot.outMotor.getCurrentPosition());
                telemetry.addData("target: ", bot.target);
            }

            // TODO: Run LiftPID test, attempt to run lift to a certain position.

            //bot.runOuttake(outPower);

            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Front left/Right", "%4.2f, %4.2f", motorPowers[0], motorPowers[2]);
            telemetry.addData("Back  left/Right", "%4.2f, %4.2f", motorPowers[2], motorPowers[3]);
            telemetry.addData("Axial,Lateral,Yaw", "%4.2f, %4.2f, %4.2f", axial, lateral, yaw);
            telemetry.addData("Intake Power", "%4.2f", intakePower);
            telemetry.addData("Lift Power left/right", "%4.2f, %4.2f", liftPower, -liftPower);
            telemetry.update();
        }

        // Stop all motion;
        bot.trainStop();
    }
}
