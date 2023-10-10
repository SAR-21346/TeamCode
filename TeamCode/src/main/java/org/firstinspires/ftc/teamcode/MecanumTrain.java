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

package org.firstinspires.ftc.teamcode;

import java.lang.Math;
import java.lang.Thread;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

public class MecanumTrain {

    // Public OpMode members.
    public DcMotor leftFrontDrive;
    public DcMotor leftBackDrive;
    public DcMotor rightFrontDrive;
    public DcMotor rightBackDrive;

    MotorController lfDriveController;
    Thread lfDriveThread;
    MotorController lbDriveController;
    Thread lbDriveThread;
    MotorController rfDriveController;
    Thread rfDriveThread;
    MotorController rbDriveController;
    Thread rbDriveThread;

    public MecanumTrain() {
    }

    HardwareMap hwMap = null;

    public void init(HardwareMap hwMapX) {
        hwMap = hwMapX;

        // Initialize the hardware variables. Note that the strings used here
        // as parameters to 'get' must correspond to the names assigned during the robot
        // configuration
        // step (using the FTC Robot Controller app).
        leftFrontDrive = hwMap.DcMotor.get("leftFrontDrive");
        leftBackDrive = hwMap.DcMotor.get("leftBackDrive");
        rightFrontDrive = hwMap.DcMotor.get("rightFrontDrive");
        rightBackDrive = hwMap.DcMotor.get("rightBackDrive");

        // Create instances of the MotorController class to run the motor asynchronously
        // in a thread
        lfDriveController = new MotorController(leftFrontDrive, DcMotor.Direction.FORWARD);
        lfDriveThread = new Thread(lfDriveController);
        lbDriveController = new MotorController(leftBackDrive, DcMotor.Direction.REVERSE);
        lbDriveThread = new Thread(lbDriveController);
        rfDriveController = new MotorController(rightFrontDrive, DcMotor.Direction.FORWARD);
        rfDriveThread = new Thread(rfDriveController);
        rbDriveController = new MotorController(rightBackDrive, DcMotor.Direction.FORWARD);
        rbDriveThread = new Thread(rbDriveController);
    }

    // trainStart()
    // Starts the motor threads
    public void trainStart() {
        lfDriveThread.start();
        lbDriveThread.start();
        rfDriveThread.start();
        rbDriveThread.start();
    }

    // calculateMotorPowers(axial, lateral, yaw)
    // axial - double
    // lateral - double
    // yaw - double
    // returns double[]
    public double[] calculateMotorPowers(double axial, double lateral, double yaw) {
        double[] motorPowers = new double[4];
        motorPowers[0] = axial + lateral + yaw;
        motorPowers[1] = axial - lateral - yaw;
        motorPowers[2] = axial - lateral + yaw;
        motorPowers[3] = axial + lateral - yaw;
        return motorPowers;
    }

    // statusToTelemetry()
    // Shows the elapsed game time and wheel power.
    public void statusToTelemetry() {
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Front left/Right", "%4.2f, %4.2f", leftFrontPower, rightFrontPower);
        telemetry.addData("Back  left/Right", "%4.2f, %4.2f", leftBackPower, rightBackPower);
        telemetry.addData("Axial,Lateral,Yaw", "%4.2f, %4.2f, %4.2f", axial, lateral, yaw);
        telemetry.update();
    }

    // trainStop()
    // Stops the motor threads
    public void trainStop() {
        lfDriveController.setPower(0);
        lbDriveController.setPower(0);
        rfDriveController.setPower(0);
        rbDriveController.setPower(0);
        lfDriveThread.interrupt();
        rfDriveThread.interrupt();
        rbDriveThread.interrupt();
        lbDriveThread.interrupt();
    }

    // MotorController
    // A class that controls a motor asynchronously in a thread
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

        public void run() {
            motor.setDirection(direction);
            while (!Thread.currentThread().isInterrupted()) {
                motor.setPower(power);
            }
            motor.setPower(0);
        }
    }
}