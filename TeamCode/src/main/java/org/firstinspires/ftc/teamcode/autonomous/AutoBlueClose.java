package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Hardware;
import org.firstinspires.ftc.teamcode.MecanumTrain;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "Auto Blue Close", group = "LinearOpMode")
//Declares as autonomous file, SDK thing
public class AutoBlueClose extends Hardware {
    ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        MecanumTrain bot = new MecanumTrain(hardwareMap, runtime);
        initHardware();
        stopMoving();

        // in milliseconds
        telemetry.addLine("Sleeping for a second");
        //close claw
        bot.closeClaw();
        telemetry.addLine("Claw closed");
        telemetry.update();
        sleep(1000);
        waitForStart();

        //Move to the side to go around the pole
        moveX(.3);
        sleep(500);

        telemetry.addLine("Moving Right");
        telemetry.update();

        clawAuto.setPosition(0);

        //Go straight
        moveY(.5);
        sleep(500);
        bot.openClaw();
    }
}