package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


@TeleOp
public class FsmTeleOp {

    public enum teleOpStates {
        TELE_START,
        TELE_GO_TO_START,
        TELE_DRIVE_TO_HUMAN_PLACED_PIXELS,
        TELE_PICK_UP_TWO,
        TELE_BACKDROP,
        TELE_PLACE_PIXELS;
    }

    teleOpStates teleOpState = teleOpStates.TELE_START;

    public void init() {

    }


    /*
     * This is the basic FSM
     * When putting this into the code put this after the while loop and waitForStart
     * Also make sure to add the conditions for all the if statements
     */

    public void loop() {
        waitForStart();
        while(opModeIsActive()) {
            switch (teleOpState) {
                case TELE_START:
                    /*check for something such as the game timer or something
                     * Makes it so that you have a beginning state and it doesn't start
                     * immediately. It's just good practice
                     */
                    teleOpState = teleOpStates.TELE_GO_TO_START;
                    break;
                case TELE_GO_TO_START:
                    if() { //Add truth statement for when the position of the robot is at start
                        teleOpState = teleOpStates.TELE_DRIVE_TO_HUMAN_PLACED_PIXELS;

                    }

                    break;
                case TELE_DRIVE_TO_HUMAN_PLACED_PIXELS:
                    // No need for truth statement in this case
                    // Add odometry to drive to correct spike

                    teleOpState = teleOpStates.TELE_PICK_UP_TWO;

                    break;
                case TELE_PICK_UP_TWO:
                    if() { // Check to see if its in right location
                        // Set motors to drop purple pixel
                        teleOpState = teleOpStates.TELE_BACKDROP;
                    }
                    break;
                case TELE_BACKDROP:
                    // No need for truth statement in this case
                    // Add odometry to drive to stack
                    teleOpState = teleOpStates.TELE_PLACE_PIXELS;

                    break;
                case TELE_PLACE_PIXELS:
                    if() { // Check to see if cameras detect pixels
                        // Set motors to pick up pixel
                        teleOpState = teleOpStates.TELE_GO_TO_START;
                    }
                    break;


            }



        }
    }
}
