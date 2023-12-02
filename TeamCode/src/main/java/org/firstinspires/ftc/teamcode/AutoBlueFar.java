package org.firstinspires.ftc.teamcode;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "Auto Blue Far", group = "LinearOpMode")
//Declares as autonomous file, SDK thing
public class AutoBlueFar extends Hardware {
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
        sleep(1000);

        //Move to the side to go around the pole
        moveX(.3);
        sleep(273);

        telemetry.addLine("Moving Right");

        //Go straight
        moveY(-.35);
        sleep(3500);
    }
}