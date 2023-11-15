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

    // Creates new object "robot" which can refer to the robot and runs trajectories
    MecanumTrain robot = new MecanumTrain(hardwareMap, new ElapsedTime());

    // Creates all the FSM States for Autonomous including all the
    // intermediate actions such as turning, strafing, etc
    // The names for the states should be self-explanatory
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


    // loop where the FSM runs
    public void loop() {
        // Creates an instance of the enum which allows the switch statement
        // to be created
        AutoStates currentState = AutoStates.AUTO_START;


        // Switch statement
        // Basically where the entire FSM is
        switch (currentState) {
            // Start Case
            case AUTO_START:
                currentState = AutoStates.AUTO_READ_SPIKE_MARK;

                break;
            case AUTO_READ_SPIKE_MARK:
                // Creates new string to store the location of the spike mark with game element
                String spikeMarkLocation;


                // If statement that allows it to pass through
                // Basically, when the FSM runs it will have to go through all the states
                // This is especially important in TeleOp so that you can have
                // an escape key
                if(spikeMarkLocation == null) {
                    if() {

                        // Right spike detected
                        spikeMarkLocation = "right";
                        currentState = AutoStates.AUTO_DRIVE_TO_SPIKE;
                        break;

                    } else if () {

                        // Middle Spike detected
                        spikeMarkLocation = "middle";
                        currentState = AutoStates.AUTO_DRIVE_TO_SPIKE;
                        break;

                    } else if () {

                        // Left spike detected
                        spikeMarkLocation = "left";
                        currentState = AutoStates.AUTO_DRIVE_TO_SPIKE;
                        break;

                    }
                } else {
                    currentState = AutoStates.AUTO_DRIVE_TO_SPIKE;
                    break;
                }

            case AUTO_DRIVE_TO_SPIKE:
                // startPose of the robot to test for where the robot is
                Pose2d startPose = new Pose2d(-35.0, 61.0, Math.toRadians(270));

                // Again, just like the if above, it is so that it can go through all
                // states in one swoop
                if(robot.getPoseEstimate() == startPose) {

                    // Right spikeMark
                    if(spikeMarkLocation.equals("right")) {

                        // Traj to rightSpike
                        Trajectory trajToRightSpike = robot.trajectoryBuilder(new Pose2d(-35.0,61.0,Math.toRadians(270)))
                                .lineToConstantHeading(new Vector2d(-46.0,38.0))
                                .build();

                        currentState = AutoStates.AUTO_OUTPUT_PURPLE;
                        break;

                        // Middle spikeMark
                    } else if (spikeMarkLocation.equals("middle")) {

                        Trajectory trajToMiddleSpike = robot.trajectoryBuilder(new Pose2d(-35.0,61.0,Math.toRadians(270)))
                                .lineToConstantHeading(new Vector2d(-24.0,35.0))
                                .build();

                        currentState = AutoStates.AUTO_OUTPUT_PURPLE;
                        break;

                        // Left spike
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

                Pose2d stackPose = new Pose2d(-56.0, 35.0, Math.toRadians(270));

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

                    Trajectory trajToBackdrop = robot.trajectoryBuilder(new Pose2d(-56.0,35.0,Math.toRadians(180)))
                            .turn(Math.toRadians(180.0)) // -------------- CHECK ----------------
                            .splineToConstantHeading(new Vector2d(49.0,35.0), Math.toRadians(0.0))
                            .build();

                    currentState = AutoStates.AUTO_OUTPUT_YELLOW;
                    break;

                } else {
                    currentState = AutoStates.AUTO_OUTPUT_YELLOW;
                    break;
                }

            case AUTO_OUTPUT_YELLOW:
                if() { // If robot still has yellow pixel
                    if(tagFoundLR == true) { // Check to see if correct qr code is read
                        // ----TO DO------- Output yellow pixel on right
                        currentState = AutoStates.AUTO_MOVE_TO_SIDE;
                        break;

                    } else if (tagFoundMR == true) {
                        // ----TO DO------- Output yellow pixel on middle
                        currentState = AutoStates.AUTO_MOVE_TO_SIDE;
                        break;
                    } else if (tagFoundRR == true) {
                        // ----TO DO------- Output yellow pixel on right
                        currentState = AutoStates.AUTO_MOVE_TO_SIDE;
                        break;
                    }
                } else {
                    currentState = AutoStates.AUTO_MOVE_TO_SIDE;
                    break;
                }
                
                break;
            case AUTO_MOVE_TO_SIDE:
                // Odometry move left or right depending on team
                Pose2d leftTagPose = new Pose2d(49.0, 41.0, Math.toRadians(0));
                Pose2d rightTagPose = new Pose2d(49.0, 29.0, Math.toRadians(0));

                if(robot.getPoseEstimate() != leftTagPose || robot.getPoseEstimate() !=rightTagPose) {
                    Trajectory trajToMid = robot.trajectoryBuilder(robot.getPoseEstimate())
                            .splineTo(new Vector2d(49.0,35.0), Math.toRadians(180))
                            .build();
                } else {
                    currentState = AutoStates.AUTO_OUTPUT_PIXEL;
                    break;
                }

            case AUTO_OUTPUT_PIXEL:
                // -----TO DO--------- set motors to output pixel
                currentState = AutoStates.AUTO_OUTPUT_YELLOW;

                break;
            case AUTO_DRIVE_TO_STACK_FOR_CYCLE:

                if(robot.getPoseEstimate() != stackPose) {
                    Trajectory trajToStackForCycle = robot.trajectoryBuilder(robot.getPoseEstimate())
                            .splineTo(new Vector2d(-24.0, 35.0), Math.toRadians(270))
                            .build();
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
                if(robot.getPoseEstimate() != backdropPose) {

                    currentState = AutoStates.AUTO_OUTPUT_PIXELS;
                    Trajectory trajToBackdropForCycle = robot.trajectoryBuilder(robot.getPoseEstimate())
                            .splineTo(new Vector2d(24,35), Math.toRadians(0))
                            .build();
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
         * The implementation here that has the if() outside and then nested if statements allow
         * it to continue to go through the entire state quickly, so setting it
         * to the origional state will not be a problem
         */
    }




}
