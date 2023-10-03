/* Copyright (c) 2021 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
//hi
//hi take 2
//hi
//HII
//hiiiiiiiiii
package org.firstinspires.ftc.teamcode;

import java.lang.Math;
import java.lang.Thread;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;



/**
 * This file contains an example of a Linear "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When a selection is made from the menu, the corresponding OpMode is executed.
 *
 * This particular OpMode illustrates driving a 4-motor Omni-Directional (or Holonomic) robot.
 * This code will work with either a Mecanum-Drive or an X-Drive train.
 * Both of these drives are illustrated at https://gm0.org/en/latest/docs/robot-design/drivetrains/holonomic.html
 * Note that a Mecanum drive must display an X roller-pattern when viewed from above.
 *
 * Also note that it is critical to set the correct rotation direction for each motor.  See details below.
 *
 * Holonomic drives provide the ability for the robot to move in three axes (directions) simultaneously.
 * Each motion axis is controlled by one Joystick axis.
 *
 * 1) Axial:    Driving forward and backward               Left-joystick Forward/Backward
 * 2) Lateral:  Strafing right and left                     Left-joystick Right and Left
 * 3) Yaw:      Rotating Clockwise and counter clockwise    Right-joystick Right and Left
 *
 * This code is written assuming that the right-side motors need to be reversed for the robot to drive forward.
 * When you first test your robot, if it moves backward when you push the left stick forward, then you must flip
 * the direction of all 4 motors (see code below).
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="Mecanum Drivetrain")
//@Disabled
public class MecanumTrain extends LinearOpMode {

    // Declare OpMode members for each of the 4 motors.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;


    private enum State {
        READY,
        NOT_READY
    }

    private State lfMotorState = State.READY;
    private State rfMotorState = State.READY;
    private State rbMotorState = State.READY;
    private State lbMotorState = State.READY;

    private static class MotorController implements Runnable {
        private final DcMotor motor; // motor definition
        private final DcMotorSimple.Direction direction; // direction of movement for a motor
                                                         // (signifies which direction it moves with positive power)
        private double power = 0;

        public MotorController(DcMotor motor, DcMotor.Direction direction) {
            this.motor = motor;
            this.direction = direction;
        }

        // setPower(power)
        // power - double
        public void setPower(double power) {
            this.power = power;
        }

        public void run(){
            motor.setDirection(direction);
            while (!Thread.currentThread().isInterrupted()) {
                motor.setPower(power);
            }
            motor.setPower(0);
        }
    }

    @Override
    public void runOpMode() {

        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        //Port 2 control hub
        leftFrontDrive  = hardwareMap.get(DcMotor.class, "FLdrive");
        //Port X control hub
        leftBackDrive  = hardwareMap.get(DcMotor.class, "BLdrive");
        //Port X control hub
        rightFrontDrive = hardwareMap.get(DcMotor.class, "FRdrive");
        //Port X control hub
        rightBackDrive = hardwareMap.get(DcMotor.class, "BRdrive");


        // Create an instance of the MotorController class "lfDriveController" to
        // run the motor asynchronously in a thread
        MotorController lfDriveController = new MotorController(leftFrontDrive, DcMotor.Direction.FORWARD);
        Thread lfDriveThread = new Thread(lfDriveController);

        // Create an instance of the MotorController class "lbDriveController" to
        // run the motor asynchronously in a thread
        MotorController lbDriveController = new MotorController(leftBackDrive, DcMotor.Direction.REVERSE);
        Thread lbDriveThread = new Thread(lbDriveController);

        // Create an instance of the MotorController class "rfDriveController" to
        // run the motor asynchronously in a thread
        MotorController rfDriveController = new MotorController(rightFrontDrive, DcMotor.Direction.FORWARD);
        Thread rfDriveThread = new Thread(rfDriveController);

        // Create an instance of the MotorController class "rbDriveController" to
        // run the motor asynchronously in a thread
        MotorController rbDriveController = new MotorController(rightBackDrive, DcMotor.Direction.FORWARD);
        Thread rbDriveThread = new Thread(rbDriveController);

        // Start each motor thread
        lfDriveThread.start();
        lbDriveThread.start();
        rfDriveThread.start();
        rbDriveThread.start();

        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            double max;

            // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
            double axial   = -gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value
            double lateral =  gamepad1.left_stick_x;
            double yaw     =  gamepad1.right_stick_x;

            // Combine the joystick requests for each axis-motion to determine each wheel's power.
            // Set up a variable for each drive wheel to save the power level for telemetry.
            double leftFrontPower  = axial + lateral + yaw;
            double rightFrontPower = axial - lateral - yaw;
            double leftBackPower   = axial - lateral + yaw;
            double rightBackPower  = axial + lateral - yaw;

            //setting minimum power of RightBack Motor, It was sticking.
            if(Math.abs(rightBackPower)<0.11&&Math.abs(rightBackPower)>0.01){
                rightBackPower=0.11;
            }
            // Normalize the values so no wheel power exceeds 100%
            // This ensures that the robot maintains the desired motion.
            max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
            max = Math.max(max, Math.abs(leftBackPower));
            max = Math.max(max, Math.abs(rightBackPower));

            if (max > 1.0) {
                leftFrontPower  /= max;
                rightFrontPower /= max;
                leftBackPower   /= max;
                rightBackPower  /= max;
            }

            // This is test code:
            //
            // Uncomment the following code to test your motor directions.
            // Each button should make the corresponding motor run FORWARD.
            //   1) First get all the motors to take to correct positions on the robot
            //      by adjusting your Robot Configuration if necessary.
            //   2) Then make sure they run in the correct direction by modifying the
            //      the setDirection() calls above.
            // Once the correct motors move in the correct direction re-comment this code.




            // Send calculated power to wheels
//          leftFrontDrive.setPower(leftFrontPower);
            lfDriveController.setPower(leftFrontPower);
//          rightFrontDrive.setPower(rightFrontPower);
            rfDriveController.setPower(rightFrontPower);
//          leftBackDrive.setPower(leftBackPower);
            lbDriveController.setPower(leftBackPower);
//          rightBackDrive.setPower(rightBackPower);
            rbDriveController.setPower(rightBackPower);

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Front left/Right", "%4.2f, %4.2f", leftFrontPower, rightFrontPower);
            telemetry.addData("Back  left/Right", "%4.2f, %4.2f", leftBackPower, rightBackPower);
            telemetry.addData("Axial,Lateral,Yaw", "%4.2f, %4.2f, %4.2f", axial, lateral, yaw);
            telemetry.update();
        }

        // .interrupt() stops the threads after the loop is done
        lfDriveThread.interrupt();
        rfDriveThread.interrupt();
        rbDriveThread.interrupt();
        lbDriveThread.interrupt();
    }}

