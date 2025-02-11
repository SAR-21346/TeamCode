package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.RobotConstants.INTAKE_POWER_NEG;
import static org.firstinspires.ftc.teamcode.RobotConstants.INTAKE_POWER_POS;
import static org.firstinspires.ftc.teamcode.RobotConstants.IntakeState.DISTANCE_CHECK;
import static org.firstinspires.ftc.teamcode.RobotConstants.IntakeState.EXTEND;
import static org.firstinspires.ftc.teamcode.RobotConstants.IntakeState.INIT;
import static org.firstinspires.ftc.teamcode.RobotConstants.IntakeState.INTAKE_ACCEPT;
import static org.firstinspires.ftc.teamcode.RobotConstants.IntakeState.INTAKE_ENABLE;
import static org.firstinspires.ftc.teamcode.RobotConstants.IntakeState.INTAKE_REJECT;
import static org.firstinspires.ftc.teamcode.RobotConstants.IntakeState.PIVOT_DOWN;
import static org.firstinspires.ftc.teamcode.RobotConstants.IntakeState.PIVOT_DOWN_BYPASS;
import static org.firstinspires.ftc.teamcode.RobotConstants.IntakeState.PIVOT_UP;
import static org.firstinspires.ftc.teamcode.RobotConstants.IntakeState.STOP;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.RobotConstants.IntakeState;

@TeleOp(name = "Red Drive")
public class RedTeleOp extends OpMode {
    MecanumTrain bot;

    Gamepad currentGamepad1, currentGamepad2, previousGamepad1, previousGamepad2;

    IntakeState intakeState;
    Timer intakeTimer, liftTimer;

    int extendDistance = 0;

    boolean buttonPressedR = false;
    boolean buttonPressedL = false;

    @Override
    public void init() {
        bot = new MecanumTrain(hardwareMap);

        intakeTimer = new Timer();

        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
    }

    @Override
    public void start() {
        setIntakeState(INIT);
    }

    @Override
    public void loop() {
        double y = -gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x;
        double rx = gamepad1.right_stick_x;

        double[] powers = bot.calculateMotorPowers(y,x,rx);
        bot.setMotorPowers(powers[0], powers[1], powers[2], powers[3], 1);

        if (gamepad2.y) {
            setIntakeState(DISTANCE_CHECK);
        }

        if (gamepad2.x) {
            setIntakeState(EXTEND);
        }

        if (gamepad2.a) {
            setIntakeState(PIVOT_DOWN_BYPASS);
        }

        if (gamepad2.b) {
            setIntakeState(STOP);
        }

        bot.encoderUpdate();
        bot.distSensorUpdate();
        intakeStateUpdate();

        telemetry.addData("current intake state", intakeState);
        telemetry.addData("intake wheel distance", bot.intakeWheelDist);
        telemetry.addData("intake wall distance", bot.intakeWallDist);
        telemetry.addData("extLPos", bot.extLPos);
        telemetry.addData("extRPos", bot.extRPos);
    }

    private void intakeStateUpdate() {
        switch (intakeState) {
            case INIT:
                bot.retract();
                bot.pivot_up();
                break;
            case DISTANCE_CHECK:
                // Check distance
                if (true) { // replace "true" with distance check
                    setIntakeState(EXTEND);
                }
                break;
            case EXTEND:
                // Extend horizontal slides
                bot.extend(5);
                setIntakeState(PIVOT_DOWN);
            case PIVOT_DOWN:
                if(bot.extRPos < 200) { // replace "true" with slide limit check
                    bot.pivot_down();
                    setIntakeState(INTAKE_ENABLE);
                }
                break;
            case PIVOT_DOWN_BYPASS:
                bot.pivot_down();
                setIntakeState(INTAKE_ENABLE);
            case INTAKE_ENABLE:
                // Pick up sample
                bot.intake.setPower(INTAKE_POWER_POS);
                bot.distSensorUpdate();
                if (bot.intakeWheelDist < 25 || bot.intakeWallDist < 20) {
                    if (bot.intakeWheelDetect() == 1 || bot.intakeWheelDetect() == 3) { // 2 for blue, 3 for yellow
                        setIntakeState(INTAKE_ACCEPT);
                    } else {
                        setIntakeState(INTAKE_REJECT);
                    }
                }
                break;
            case INTAKE_REJECT:
                bot.intake.setPower(INTAKE_POWER_NEG);
                if (intakeTimer.getElapsedTimeSeconds() > 0.4) {
                    setIntakeState(INTAKE_ENABLE);
                }
                break;
            case INTAKE_ACCEPT:
                bot.distSensorUpdate();
                if (bot.intakeWallDist < 30) {
                    bot.intake.setPower(0);
                    setIntakeState(PIVOT_UP);
                }
            case PIVOT_UP:
                // pivot intake up
                bot.pivot_up();
                // retract slides
                bot.retract();
                bot.intake.setPower(0);
                if (bot.extRPos > 200) {
                    setIntakeState(STOP);
                }
                break;
            case STOP:
                // pivot intake up
                bot.pivot_up();
                // retract slides
                bot.retract();
                bot.intake.setPower(0);
                break;
        }
    }

    private void setIntakeState(IntakeState iState) {
        intakeState = iState;
        intakeTimer.resetTimer();
        intakeStateUpdate();
    }
}
