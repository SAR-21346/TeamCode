package org.firstinspires.ftc.teamcode.autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Hardware;
import org.firstinspires.ftc.teamcode.MecanumTrain;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "Auto Blue Far", group = "LinearOpMode")
//Declares as autonomous file, SDK thing
public class AutoBlueFar extends Hardware {
    ElapsedTime runtime = new ElapsedTime();
    MecanumTrain bot;
    @Override
    public void runOpMode() throws InterruptedException {
        bot = new MecanumTrain(hardwareMap, runtime);

        initHardware();
        stopMoving();

        // in milliseconds
        telemetry.addLine("Sleeping for a second");

        //close claw
        bot.closeClaw();
        sleep(1000);
        waitForStart();

        //Move to the side to go around the pole
        moveX(.3);
        sleep(273);

        telemetry.addLine("Moving Right");

        //Go straight
        moveY(-.35);
        sleep(3000);
    }
}