package org.firstinspires.ftc.teamcode.autonomous;

import static org.firstinspires.ftc.teamcode.RobotConstants.IntakeState.INTAKE_FLIP_IN;
import static org.firstinspires.ftc.teamcode.RobotConstants.IntakeState.INTAKE_INIT;
import static org.firstinspires.ftc.teamcode.RobotConstants.IntakeState.INTAKE_RELEASE;
import static org.firstinspires.ftc.teamcode.RobotConstants.IntakeState.INTAKE_RETRACT;
import static org.firstinspires.ftc.teamcode.RobotConstants.IntakeState.INTAKE_SAMPLE_IN;
import static org.firstinspires.ftc.teamcode.RobotConstants.IntakeState.INTAKE_SPIN;
import static org.firstinspires.ftc.teamcode.RobotConstants.IntakeState.INTAKE_START;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.MecanumTrain;
import org.firstinspires.ftc.teamcode.RobotConstants.IntakeState;
import org.firstinspires.ftc.teamcode.pedroPathing.util.Timer;

public class BlueBasketNoSpec extends OpMode {

    private IntakeState intakeState;
    private Timer intakeTimer, pathTimer;
    private ElapsedTime opmodeTimer;

    private MecanumTrain bot;

    private int pathState;

    @Override
    public void init() {
        intakeTimer = new Timer();
        pathTimer = new Timer();
        opmodeTimer = new ElapsedTime();

    }

    @Override
    public void start() {
        opmodeTimer.reset();

        bot = new MecanumTrain(hardwareMap, opmodeTimer);

        intakeState = INTAKE_INIT;
        pathState = 0;
    }
    @Override
    public void loop() {

    }

    private void buildPaths() {

    }

    private void autonomousPathUpdate() {

    }

    private void setPathState() {

    }

    private void setIntakeState (IntakeState iState) {
        intakeState = iState;
        intakeTimer.resetTimer();
        intakeStateUpdate();
    }

    private void intakeStateUpdate () {
        switch (intakeState) {
            case INTAKE_INIT:
                bot.setIntakeServo("off");
                bot.setIntakePivot("in");
                bot.setHorizontalExtension("in");
            case INTAKE_FLIP_OUT:
                bot.setIntakePivot("out");
                if (intakeTimer.getElapsedTimeSeconds() > 1.0) {
                    setIntakeState(INTAKE_SPIN);
                }
                break;
            case INTAKE_FLIP_IN:
                bot.setIntakePivot("in");
                if (bot.horizontalLimit.isPressed()) {
                    intakeTimer.resetTimer();
                    if (intakeTimer.getElapsedTimeSeconds() > 4) {
                        bot.setIntakeServo("backward");
                    }
                }
                setIntakeState(INTAKE_RELEASE);
                break;
            case INTAKE_SPIN:
                bot.setIntakeServo("forward");
                if (sampleDetected()) {
                    setIntakeState(INTAKE_SAMPLE_IN);
                }
                break;
            case INTAKE_SAMPLE_IN:
                bot.setIntakeServo("off");
                int r = bot.intakeColor.red(), g = bot.intakeColor.green(), b = bot.intakeColor.blue();
                int maxValue = Math.max(r, Math.max(g, b));
                if (sampleDetected() && maxValue == r) {
                    bot.setIntakeServo("backward");
                    setIntakeState(INTAKE_SPIN);
                }
                setIntakeState(INTAKE_RETRACT);
                break;
            case INTAKE_RETRACT:
                bot.setHorizontalExtension("in");
                if (intakeTimer.getElapsedTimeSeconds() > 0.8) {
                    setIntakeState(INTAKE_FLIP_IN);
                }
                break;
            case INTAKE_RELEASE:
                bot.setIntakePivot("in");
                bot.setIntakeServo("backward");
                setIntakeState(INTAKE_START);
                break;
            case INTAKE_STOP:
                bot.setIntakeServo("off");
                bot.setIntakePivot("in");
                bot.setHorizontalExtension("in");

                break;
        }
    }

    private boolean sampleDetected() {
        if (bot.intakeColor instanceof DistanceSensor) {
            ColorSensor color = bot.intakeColor;
            double distance = ((DistanceSensor) color).getDistance(DistanceUnit.MM);
            if (distance < 30) {
                return true;
            }
        }
        return false;
    }
}
