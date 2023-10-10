package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.MecanumTrain;
import org.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "TeleOpMain")
public class TeleOpMain {
    private ElapsedTime runtime = new ElapsedTime();

    // initialize motors
    private DcMotor lfMotor = hardwareMap.get(DcMotor.class, "leftFrontDrive");
    private DcMotor lbMotor = hardwareMap.get(DcMotor.class, "leftBackDrive");
    private DcMotor rbMotor = hardwareMap.get(DcMotor.class, "rightBackDrive");
    private DcMotor rfMotor = hardwareMap.get(DcMotor.class, "rightFrontDrive");

    private MecanumTrain mecanumTrain = new MecanumTrain(lfMotor, lbMotor, rfMotor, rbMotor);

    public void runOpMode() {

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
            mecanumTrain.getLeftFrontDrive().setPower(motorPowers[0]);
            mecanumTrain.getLeftBackDrive().setPower(motorPowers[1]);
            mecanumTrain.getRightFrontDrive().setPower(motorPowers[2]);
            mecanumTrain.getRightBackDrive().setPower(motorPowers[3]);

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Front left/Right", "%4.2f, %4.2f", leftFrontPower, rightFrontPower);
            telemetry.addData("Back  left/Right", "%4.2f, %4.2f", leftBackPower, rightBackPower);
            telemetry.addData("Axial,Lateral,Yaw", "%4.2f, %4.2f, %4.2f", axial, lateral, yaw);
            telemetry.update();
        }
    }
}
