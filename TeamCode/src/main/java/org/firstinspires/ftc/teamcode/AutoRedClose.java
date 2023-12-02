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
        //close claw
        clawAuto.setPosition(0);
        sleep(1000);
        waitForStart();

        //Move to the side to go around the pole
        moveX(.3 * -1);
        sleep(1000);

        telemetry.addLine("Moving Right");

        clawAuto.setPosition(0);

        //Go straight
        moveY(.5);
        sleep(750);
        clawAuto.setPosition(0.2);
    }
}