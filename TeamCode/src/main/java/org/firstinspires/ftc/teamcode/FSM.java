package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;


@Autonomous
public class FSM {

    // all the states for autonomous

    MecanumTrain robot = new MecanumTrain(hardwareMap, new ElapsedTime());

    public enum AutoStates {
        AUTO_START,
        AUTO_READ_SPIKE_MARK,
        AUTO_DRIVE_TO_SPIKE,
        AUTO_OUTPUT_PURPLE,
        AUTO_DRIVE_TO_STACK,
        AUTO_PICK_UP_PIXEL,
        AUTO_DRIVE_TO_BACKDROP,
        AUTO_OUTPUT_YELLOW,
        AUTO_MOVE_TO_SIDE,
        AUTO_OUTPUT_PIXEL,
        AUTO_DRIVE_TO_STACK_FOR_CYCLE,
        AUTO_PICK_UP_TWO,
        AUTO_DRIVE_TO_BACKDROP_FOR_CYCLE,
        AUTO_OUTPUT_PIXELS;
    }

    /* this is just the switch statement with all the cases
     * so make sure to add this to the while loop in the main opmode
     */

    public void loop() {
        AutoStates currentState = AutoStates.AUTO_START;

        switch (currentState) {
            case AUTO_START:
                /*check for something such as the game timer or something
                * Makes it so that you have a beginning state and it doesn't start
                * immediately. It's just good practice
                */
                currentState = AutoStates.AUTO_READ_SPIKE_MARK;

                break;
            case AUTO_READ_SPIKE_MARK:
                String spikeMarkLocation; // The location of the spike mark is realtive to the robot

                if(spikeMarkLocation == null) {
                    if() { //Add truth statement for which spike mark is detected

                        spikeMarkLocation = "right";
                        currentState = AutoStates.AUTO_DRIVE_TO_SPIKE;
                        break;

                    } else if () {

                        spikeMarkLocation = "middle";
                        currentState = AutoStates.AUTO_DRIVE_TO_SPIKE;
                        break;

                    } else if () {

                        spikeMarkLocation = "left";
                        currentState = AutoStates.AUTO_DRIVE_TO_SPIKE;
                        break;

                    }
                } else {
                    currentState = AutoStates.AUTO_DRIVE_TO_SPIKE;
                    break;
                }

            case AUTO_DRIVE_TO_SPIKE:
                Pose2d startPose = new Pose2d(-35.0, 61.0, Math.toRadians(270));
                if(robot.getPoseEstimate() == startPose) {

                    if(spikeMarkLocation.equals("right")) {

                        Trajectory trajToRightSpike = robot.trajectoryBuilder(new Pose2d(-35.0,61.0,Math.toRadians(270)))
                                .lineToConstantHeading(new Vector2d(-46.0,38.0))
                                .build();

                        currentState = AutoStates.AUTO_OUTPUT_PURPLE;
                        break;

                    } else if (spikeMarkLocation.equals("middle")) {

                        Trajectory trajToMiddleSpike = robot.trajectoryBuilder(new Pose2d(-35.0,61.0,Math.toRadians(270)))
                                .lineToConstantHeading(new Vector2d(-24.0,35.0))
                                .build();

                        currentState = AutoStates.AUTO_OUTPUT_PURPLE;
                        break;

                    } else if (spikeMarkLocation.equals("left")) {

                        Trajectory trajToMiddleSpike = robot.trajectoryBuilder(new Pose2d(-35.0,61.0,Math.toRadians(270)))
                                .lineToConstantHeading(new Vector2d(-24.0,38.0))
                                .build();

                        currentState = AutoStates.AUTO_OUTPUT_PURPLE;
                        break;

                    }

                }

            case AUTO_OUTPUT_PURPLE:
                Pose2d middleSpikePose = new Pose2d(-24.0, 35.0, Math.toRadians(270));
                if(robot.getPoseEstimate() == middleSpikePose) { // Check to see if its in right location
                    // --------------------------TO DO-----------------Set motors to drop purple pixel
                    currentState = AutoStates.AUTO_DRIVE_TO_STACK;
                    break;
                } else {
                    currentState = AutoStates.AUTO_DRIVE_TO_STACK;
                    break;
                }

            case AUTO_DRIVE_TO_STACK:

                Pose2d stackPose = new Pose2d(-24.0, 35.0, Math.toRadians(270));

                if(robot.getPoseEstimate() != stackPose) {

                    if(spikeMarkLocation.equals("right")) {

                        Trajectory trajToStackFromRight = robot.trajectoryBuilder(new Pose2d(-46.0,38.0,Math.toRadians(270)))
                                .splineTo(new Vector2d(-56.0,35.0), Math.toRadians(180))
                                .build();

                        currentState = AutoStates.AUTO_PICK_UP_PIXEL;
                        break;

                    } else if (spikeMarkLocation.equals("middle")) {

                        Trajectory trajToStackFromRight = robot.trajectoryBuilder(new Pose2d(-24.0,35.0,Math.toRadians(270)))
                                .splineTo(new Vector2d(-56.0,35.0), Math.toRadians(180))
                                .build();

                        currentState = AutoStates.AUTO_PICK_UP_PIXEL;
                        break;

                    } else if (spikeMarkLocation.equals("left")) {

                        Trajectory trajToStackFromRight = robot.trajectoryBuilder(new Pose2d(-24.0,38.0,Math.toRadians(270)))
                                .splineTo(new Vector2d(-56.0,35.0), Math.toRadians(180))
                                .build();

                        currentState = AutoStates.AUTO_PICK_UP_PIXEL;
                        break;

                    }

                } else {
                    currentState = AutoStates.AUTO_PICK_UP_PIXEL;
                    break;
                }



            case AUTO_PICK_UP_PIXEL:
                if() { // Check to see if cameras detect pixels
                    // --------------------------TO DO--------- Set motors to pick up pixel
                    currentState = AutoStates.AUTO_DRIVE_TO_BACKDROP;
                }
                break;
            case AUTO_DRIVE_TO_BACKDROP:

                Pose2d backdropPose = new Pose2d(-24.0, 35.0, Math.toRadians(270));

                if(robot.getPoseEstimate() != backdropPose) {

                    Trajectory trajToBackdrop = robot.trajectoryBuilder(new Pose2d(-24.0,38.0,Math.toRadians(270)))
                            .splineTo(new Vector2d(49.0,35.0), Math.toRadians(180))
                            .build();

                    currentState = AutoStates.AUTO_OUTPUT_YELLOW;
                    break;

                } else {
                    currentState = AutoStates.AUTO_OUTPUT_YELLOW;
                    break;
                }

            case AUTO_OUTPUT_YELLOW:
                if() { // Check to see if correct qr code is read
                    // Set motors to output yellow
                    currentState = AutoStates.AUTO_MOVE_TO_SIDE;
                }
                break;
            case AUTO_MOVE_TO_SIDE:
                // Odometry move left or right depending on team
                currentState = AutoStates.AUTO_OUTPUT_PIXEL;

                break;
            case AUTO_OUTPUT_PIXEL:
                // set motors to output pixel
                currentState = AutoStates.AUTO_OUTPUT_YELLOW;

                break;
            case AUTO_DRIVE_TO_STACK_FOR_CYCLE:
                if() { // Check to see if two pixels have been output
                    // Use odometry to drive to stack
                    currentState = AutoStates.AUTO_PICK_UP_TWO;
                }
                break;
            case AUTO_PICK_UP_TWO:
                if() { // Check to see if two pixels are there
                    // set motors to pick up two pixels
                    currentState = AutoStates.AUTO_DRIVE_TO_BACKDROP_FOR_CYCLE;
                }
                break;
            case AUTO_DRIVE_TO_BACKDROP_FOR_CYCLE:
                if() { // Check to see if two pixels are there
                    // Odometry to drive to backdrop
                    currentState = AutoStates.AUTO_OUTPUT_PIXELS;
                }
                break;
            case AUTO_OUTPUT_PIXELS:
                if() { // Check to see if the bot is in front of the board
                    // Set motors to place pixel
                    currentState = AutoStates.AUTO_DRIVE_TO_STACK_FOR_CYCLE;
                }
                break;

        }

        /* **********Note***********
         * When implementing this in teleOp, use an escape key, like Y that allows the
         * entire thing to stop, and you can fix anything before starting the new cycle
         * ALSO, for the escape key, set the state to be the start state, making it restart
         * the entire cycle
         */
    }




}
/*
@@ -1,12 +1,21 @@
package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.*;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;


@Autonomous
public class FSM {

    // all the states for autonomous



    public enum AutoStates {
        AUTO_START,
        AUTO_READ_SPIKE_MARK,
@ -28,6 +37,7 @@ public class FSM {
     * so make sure to add this to the while loop in the main opmode



    public void loop() {
        AutoStates currentState = AutoStates.AUTO_START;

        @ -40,15 +50,29 @@ public class FSM {
            currentState = AutoStates.AUTO_READ_SPIKE_MARK;
                break;
            case AUTO_READ_SPIKE_MARK:
                    if() { //Add truth statement for which spike mark is detected
                currentState = AutoStates.AUTO_DRIVE_TO_SPIKE;
                // Add variable storing the right spike mark location
            }
            // Check what the spike mark is and store it in a variable

                break;
            case AUTO_DRIVE_TO_SPIKE:
                    // No need for truth statement in this case
                    // Add odometry to drive to correct spike
                    if(/* Marker #1  {
                TrajectoryBuilder trajToSpikeOne = new TrajectoryBuilder(new Vector2d())
                        .splineTo(new Vector2d(/* Location of the 2nd spike mark ), Math.toRadians(180))
                        .build();
                currentState = AutoStates.AUTO_DRIVE_TO_SPIKE;
            } else if ( Marker #2 ) {
                TrajectoryBuilder trajToSpikeTwo = new TrajectoryBuilder(new Vector2d(10.0,10.0))
                        .lineToSplineHeading(new Vector2d(10.0,10.0))
                        .build();

            } else-if ( Marker #3 ) {

                TrajectoryBuilder trajToSpikeThree = new TrajectoryBuilder(new Vector2d())
                        .splineTo(new Vector2d(/* Location of the 3rd spike mark ), Math.toRadians(180))
                        .build();

            }

            currentState = AutoStates.AUTO_OUTPUT_PURPLE;

            @ -126,9 +150,12 @@ public class FSM {
         * ALSO, for the escape key, set the state to be the start state, making it restart
         * the entire cycle


            }






        }
 */