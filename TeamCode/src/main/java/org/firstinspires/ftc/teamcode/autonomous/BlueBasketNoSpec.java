package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.MecanumTrain;
import org.firstinspires.ftc.teamcode.pedroPathing.util.Timer;

@Autonomous(name="Blue Basket No Specimen Preload + 3")
public class BlueBasketNoSpec extends OpMode {

    private Timer pathTimer, intakeTimer;
    private ElapsedTime opmodeTimer;
    private MecanumTrain bot;

    @Override
    public void init() {
        pathTimer = new Timer();
        intakeTimer = new Timer();
        opmodeTimer = new ElapsedTime();

        opmodeTimer.reset();
        bot = new MecanumTrain(hardwareMap, opmodeTimer);
    }

    @Override
    public void loop() {
        bot.follower.update();
    }

    @Override
    public void start() {

    }

}
