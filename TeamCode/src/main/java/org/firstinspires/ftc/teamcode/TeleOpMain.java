package org.firstinspires.ftc.teamcode;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@TeleOp(name = "TeleOpMain")
public class TeleOpMain extends LinearOpMode{
    private final ElapsedTime runtime = new ElapsedTime();
    MecanumTrain bot;

    static final double SPEED_MULTIPLIER = 0.50 ;

    @Override
    public void runOpMode() throws InterruptedException {
        //bot initialization
        bot = new MecanumTrain(hardwareMap, runtime);

        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        bot.verticalExtension.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        double intakePower = 0.0;
        int liftPos = 0;
        double servoPos = 0.3;

        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();
        runtime.reset();


        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            // get controller inputs
            double gamepadLS_Y_adj = Math.abs(gamepad1.left_stick_y) < .10 ? 0 : gamepad1.left_stick_y;

            double axial = -gamepadLS_Y_adj; // Note: pushing stick forward gives negative value
            double lateral = gamepad1.left_stick_x;
            double yaw = gamepad1.right_stick_x;

            // calculate motor powers
            double[] motorPowers = bot.calculateMotorPowers(axial, lateral, yaw);

            if (gamepad1.right_bumper) { bot.setMotorPowers(motorPowers[0], motorPowers[1], motorPowers[2], motorPowers[3], SPEED_MULTIPLIER); }
            else { bot.setMotorPowers(motorPowers[0], motorPowers[1], motorPowers[2], motorPowers[3], 1); }

            telemetry.addLine("ENCODER:");
            telemetry.addData("Left Parallel Encoder", bot.leftFrontDrive.getCurrentPosition());
            telemetry.addData("Right Parallel Encoder", bot.rightFrontDrive.getCurrentPosition());
            telemetry.addLine();
            telemetry.addData("Status", "Run Time: " + runtime);
            telemetry.addData("Front left/Right", "%4.2f, %4.2f", motorPowers[0], motorPowers[2]);
            telemetry.addData("Back  left/Right", "%4.2f, %4.2f", motorPowers[2], motorPowers[3]);
            telemetry.addData("Axial,Lateral,Yaw", "%4.2f, %4.2f, %4.2f", axial, lateral, yaw);
            telemetry.addData("Intake Power", "%4.2f", intakePower);
            telemetry.update();


        }
    }
}
