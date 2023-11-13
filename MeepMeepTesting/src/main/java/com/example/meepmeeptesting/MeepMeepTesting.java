package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
        private double kp; // Proportional gain
        private double ki; // Integral gain
        private double kd; // Derivative gain

        private double setpoint; // Desired position
        private double integral = 0; // Integral accumulator
        private double prevError = 0; // Previous error

        public MeepMeepTesting(double kp, double ki, double kd, double setpoint) {
            this.kp = kp;
            this.ki = ki;
            this.kd = kd;
            this.setpoint = setpoint;
        }

        public double calculateControlSignal(double currentPosition) {
            double error = setpoint - currentPosition;

            // Proportional term
            double proportional = kp * error;

            // Integral term (with anti-windup)
            integral += ki * error;
            if (integral > 10) { // Add anti-windup (adjust as needed)
                integral = 10;
            } else if (integral < -10) {
                integral = -10;
            }

            // Derivative term
            double derivative = kd * (error - prevError);

            prevError = error;

            return proportional + integral + derivative;
        }

        public static void main(String[] args) {
            double kp = 0.1; // Tune these values as needed
            double ki = 0.01;
            double kd = 0.01;
            double setpoint = 0.0; // Desired position

            MeepMeepTesting pidController = new MeepMeepTesting(kp, ki, kd, setpoint);

            // Simulated robot arm
            double currentPosition = 5.0; // Initial position (you would replace this with actual sensor data)

            for (int i = 0; i < 100; i++) {
                double controlSignal = pidController.calculateControlSignal(currentPosition);

                System.out.println("Control signal: " + controlSignal);
                // Apply controlSignal to the robot arm actuators
                // Here, you would send the control signal to the motors or actuators of your robot arm

                // Simulate time passing and update the position
                currentPosition += 0.1; // Replace with actual position feedback from your robot arm
            }
        }
    }


//    public static void main(String[] args) {
//        MeepMeep meepMeep = new MeepMeep(800);
//
//        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
//                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
//                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
//                .followTrajectorySequence(drive ->
//                        drive.trajectorySequenceBuilder(new Pose2d(0, 0, 0))
//                                .forward(30)
//                                .turn(Math.toRadians(90))
//                                .forward(30)
//                                .turn(Math.toRadians(90))
//                                .forward(30)
//                                .turn(Math.toRadians(90))
//                                .forward(30)
//                                .turn(Math.toRadians(90))
//                                .build()
//                );
//
//        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
//                .setDarkMode(true)
//                .setBackgroundAlpha(0.95f)
//                .addEntity(myBot)
//                .start();
//    }
