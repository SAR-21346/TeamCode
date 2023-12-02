package org.firstinspires.ftc.teamcode;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "Auto Red Close", group = "LinearOpMode")
//Declares as autonomous file, SDK thing
public class AutoRedClose extends Hardware {
    @Override
    public void runOpMode() throws InterruptedException {
        initHardware();
        stopMoving();

        // in milliseconds
        telemetry.addLine("Sleeping for a second");
        sleep(1000);
        waitForStart();

        //close claw and sleep for .5 seconds
        clawAuto.setPosition(0);
        sleep(500);

        //Move to the side to go around the pole
        moveX(.3 * -1);
        sleep(1000);

        telemetry.addLine("Moving Right");

        //Go straight
        moveY(.5 * -1);
        sleep(250);
    }
}