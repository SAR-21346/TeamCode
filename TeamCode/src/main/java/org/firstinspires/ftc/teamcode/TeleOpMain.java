package org.firstinspires.ftc.teamcode;


import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.MecanumTrain;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "TeleOpMain")
public class TeleOpMain extends LinearOpMode{
    private final ElapsedTime runtime = new ElapsedTime();
    MecanumTrain bot;

    static final double SPEED_MULTIPLIER = 0.45 ;

    @Override
    public void runOpMode() throws InterruptedException {
        //bot initialization
        bot = new MecanumTrain(hardwareMap, runtime);

        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        bot.leftSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bot.rightSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        bot.arm_start = bot.outMotor.getCurrentPosition();
        bot.liftL_start = bot.leftSlide.getCurrentPosition();
        bot.liftR_start = bot.rightSlide.getCurrentPosition();

        double intakePower = 0.0;
        int liftPos = 0;
        double servoPos = 0.3;
        bot.target = 0;

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

//            if (gamepad2.dpad_down) { intakePower = .4; }
//            if (gamepad2.dpad_up) { intakePower = -.4; }
//            if (gamepad2.dpad_left) { intakePower = 0; }

            if (gamepad2.triangle) {
                sleep(50);
                liftPos += 20;
            }
            if (gamepad2.cross) {
                sleep(50);
                liftPos -= 20;
            }
            if (gamepad2.square) {
                liftPos = 0;
            }

            if(gamepad2.left_trigger > 0){
                float decreaseFlip = gamepad2.left_trigger;
                if (bot.target >= 50 && bot.target <=150) {
                    bot.target -= decreaseFlip*5;
                } else if (bot.target <=50 && bot.target >= -20){
                    bot.target -= decreaseFlip*3;
                }
            }

            if (gamepad2.right_trigger > 0) {
                float increaseFlip = gamepad2.right_trigger;
                if (bot.target <= 100 && bot.target >-30) {
                    bot.target += increaseFlip*5;
                }
                if (bot.target >100 && bot.target <=130) {
                    bot.target += increaseFlip*3;
                }
            }

            if (gamepad1.left_bumper && gamepad1.dpad_right) {
                bot.openDrone();
                sleep(1000);
                bot.closeDrone();
            }

            if (gamepad2.left_bumper) {
                bot.closeClaw();
                intakePower = 0;
            }
            if (gamepad2.right_bumper) {
                bot.openClaw();
                intakePower = -0.4;
            }



            // calculate motor powers
            double[] motorPowers = bot.calculateMotorPowers(axial, lateral, yaw);

            if (gamepad1.right_bumper) {
                bot.setMotorPowers(motorPowers[0] * SPEED_MULTIPLIER,
                                  motorPowers[1] * SPEED_MULTIPLIER,
                                  motorPowers[2] * SPEED_MULTIPLIER,
                                  motorPowers[3] * SPEED_MULTIPLIER);
            } else {
                bot.setMotorPowers(motorPowers[0], motorPowers[1], motorPowers[2], motorPowers[3]);
            }



            bot.runIntake(intakePower);
            bot.runLift(liftPos);
            bot.updateArmPID(bot.outMotor.getCurrentPosition());

            telemetry.addLine("ENCODER:");
            telemetry.addData("leftPos", bot.leftFrontDrive.getCurrentPosition());
            telemetry.addData("rightPos", bot.rightFrontDrive.getCurrentPosition());
            telemetry.addData("frontPos", bot.spinTake.getCurrentPosition());
            //bot.runOuttake(outPower);
            telemetry.addData("ArmPos: ", bot.outMotor.getCurrentPosition());
            telemetry.addData("target: ", bot.target);
            telemetry.addLine();
            telemetry.addData("Servo: ", bot.drone.getPosition());
            telemetry.addData("Left Slide Pos: ", bot.leftSlide.getCurrentPosition());
            telemetry.addData("Right Slide Pos: ", bot.rightSlide.getCurrentPosition());
            telemetry.addLine();
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Front left/Right", "%4.2f, %4.2f", motorPowers[0], motorPowers[2]);
            telemetry.addData("Back  left/Right", "%4.2f, %4.2f", motorPowers[2], motorPowers[3]);
            telemetry.addData("Axial,Lateral,Yaw", "%4.2f, %4.2f, %4.2f", axial, lateral, yaw);
            telemetry.addData("Intake Power", "%4.2f", intakePower);
            telemetry.update();


        }
    }
}
