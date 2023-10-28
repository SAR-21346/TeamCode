package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous
public class FSM {

    // all the states for autonomous

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
                if() { //Add truth statement for which spike mark is detected
                    currentState = AutoStates.AUTO_DRIVE_TO_SPIKE;
                    // Add variable storing the right spike mark location
                }

                break;
            case AUTO_DRIVE_TO_SPIKE:
                // No need for truth statement in this case
                // Add odometry to drive to correct spike

                currentState = AutoStates.AUTO_OUTPUT_PURPLE;

                break;
            case AUTO_OUTPUT_PURPLE:
                if() { // Check to see if its in right location
                    // Set motors to drop purple pixel
                    currentState = AutoStates.AUTO_DRIVE_TO_STACK;
                }
                break;
            case AUTO_DRIVE_TO_STACK:
                // No need for truth statement in this case
                // Add odometry to drive to stack
                currentState = AutoStates.AUTO_PICK_UP_PIXEL;

                break;
            case AUTO_PICK_UP_PIXEL:
                if() { // Check to see if cameras detect pixels
                    // Set motors to pick up pixel
                    currentState = AutoStates.AUTO_DRIVE_TO_BACKDROP;
                }
                break;
            case AUTO_DRIVE_TO_BACKDROP:
                if() { // Check to see if two pixels have been picked up
                    // Use odometry to drive to backdrop
                    currentState = AutoStates.AUTO_OUTPUT_YELLOW;
                }
                break;
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
