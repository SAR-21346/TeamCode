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


import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;


public class MecanumTrain {
    // Public OpMode members.
    public DcMotor leftFrontDrive;
    public DcMotor leftBackDrive;
    public DcMotor rightFrontDrive;
    public DcMotor rightBackDrive;
    public DcMotor leftSlide;
    public DcMotor rightSlide;
    public DcMotor spinTake;
    public DcMotor outMotor;
    public OpenCvCamera camera;
    public AprilTagDetectionPipeline aprilTagDetectionPipeline;

    // MotorController instances and threads
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
        leftFrontDrive = hwMap.get(DcMotor.class, "FLdrive");
        leftBackDrive = hwMap.get(DcMotor.class, "BLdrive");
        rightFrontDrive = hwMap.get(DcMotor.class, "FRdrive");
        rightBackDrive = hwMap.get(DcMotor.class, "BRdrive");
        leftSlide = hwMap.get(DcMotor.class, "Lslide");
        rightSlide = hwMap.get(DcMotor.class, "Rslide");
        spinTake = hwMap.get(DcMotor.class, "Spintake");
        outMotor = hwMap.get(DcMotor.class, "Outtake");

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

        // Camera Initialization
        int cameraMonitorViewId = hwMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id",
                hwMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hwMap.get(WebcamName.class, "camera"),
                cameraMonitorViewId);
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

    //Create States for motors
    private enum State {
        READY,
        NOT_READY
    }

    //Set the states of all motors to ready
    private State lfMotorState = State.READY;
    private State rfMotorState = State.READY;
    private State rbMotorState = State.READY;
    private State lbMotorState = State.READY;
    private State lSlMotorState = State.READY;
    private State rSlMotorState = State.READY;
    private State inMotorState = State.READY;
    private State outMotorState = State.READY;


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

        //motor definition
        private final DcMotor motor;

        // direction of movement for a motor
        // (signifies which direction it moves with positive power)
        private final DcMotorSimple.Direction direction;

        //Initial Power set to Zero
        private double power = 0;

        public MotorController(DcMotor motor, DcMotorSimple.Direction direction) {
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
