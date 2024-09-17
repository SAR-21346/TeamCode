package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.RobotConstants.IntakeState.INTAKE_EXTEND;
import static org.firstinspires.ftc.teamcode.RobotConstants.IntakeState.INTAKE_FLIP_IN;
import static org.firstinspires.ftc.teamcode.RobotConstants.IntakeState.INTAKE_FLIP_OUT;
import static org.firstinspires.ftc.teamcode.RobotConstants.IntakeState.INTAKE_RELEASE;
import static org.firstinspires.ftc.teamcode.RobotConstants.IntakeState.INTAKE_RETRACT;
import static org.firstinspires.ftc.teamcode.RobotConstants.IntakeState.INTAKE_SAMPLE_IN;
import static org.firstinspires.ftc.teamcode.RobotConstants.IntakeState.INTAKE_SAMPLE_OUT;
import static org.firstinspires.ftc.teamcode.RobotConstants.IntakeState.INTAKE_SPIN;
import static org.firstinspires.ftc.teamcode.RobotConstants.IntakeState.INTAKE_START;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.RobotConstants.IntakeState;
import org.firstinspires.ftc.teamcode.pedroPathing.util.Timer;

public class TeleOpFSMBlue extends OpMode {
    MecanumTrain bot;

    private IntakeState intakeState;
    private Timer intakeTimer;
    private ElapsedTime opmodeTimer;
    private final double SPEED_MULTIPLIER = 0.50;

    private void intakeStateUpdate () {
        switch (intakeState) {
            case INTAKE_START:
                bot.setIntakeServo("off");
                bot.setIntakePivot("in");
                bot.setHorizontalExtension("in");

                if ((bot.leftFrontDist.getDistance(DistanceUnit.CM) + bot.rightFrontDist.getDistance(DistanceUnit.CM)) / 2 < 5) {
                    setIntakeState(INTAKE_EXTEND);
                }
                break;
            case INTAKE_EXTEND:
                bot.setHorizontalExtension("out");
                setIntakeState(INTAKE_FLIP_OUT);
                break;
            case INTAKE_FLIP_OUT:
                bot.setIntakePivot("out");
                setIntakeState(INTAKE_SPIN);
                break;
            case INTAKE_FLIP_IN:
                bot.setIntakePivot("in");
                setIntakeState(INTAKE_RETRACT);
                break;
            case INTAKE_SPIN:
                bot.setIntakeServo("forward");
                setIntakeState(INTAKE_SAMPLE_IN);
                break;
            case INTAKE_SAMPLE_IN:
                bot.setIntakeServo("off");
                if((bot.intakeColor.blue() > 1) || bot.intakeColor.alpha() > 1) {
                    setIntakeState(INTAKE_RETRACT);
                } else if (bot.intakeColor.red() > 1) {
                    setIntakeState(INTAKE_SAMPLE_OUT);
                }
                break;
            case INTAKE_SAMPLE_OUT:
                break;
            case INTAKE_RETRACT:
                bot.setHorizontalExtension("in");
                if (bot.horizontalLimit.isPressed()) {
                    setIntakeState(INTAKE_FLIP_IN);
                }
                setIntakeState(INTAKE_RELEASE);
                break;
            case INTAKE_RELEASE:
                bot.setIntakePivot("out");
                setIntakeState(INTAKE_START);
                break;
        }
    }



    public void setIntakeState (IntakeState iState) {
        intakeState = iState;
        intakeTimer.resetTimer();
        intakeStateUpdate();
    }



    @Override
    public void init() {
        intakeTimer = new Timer();
        opmodeTimer = new ElapsedTime();

        opmodeTimer.reset();

        bot = new MecanumTrain(hardwareMap, opmodeTimer);
        bot.verticalExtension.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        intakeState = INTAKE_START;
    }

    @Override
    public void loop() {
        intakeStateUpdate();

        double gamepadLS_Y_adj = Math.abs(gamepad1.left_stick_y) < .10 ? 0 : gamepad1.left_stick_y;

        double axial = -gamepadLS_Y_adj; // Note: pushing stick forward gives negative value
        double lateral = gamepad1.left_stick_x;
        double yaw = gamepad1.right_stick_x;

        // calculate motor powers
        double[] motorPowers = bot.calculateMotorPowers(axial, lateral, yaw);

        if (gamepad1.right_bumper) { bot.setMotorPowers(motorPowers[0], motorPowers[1], motorPowers[2], motorPowers[3], SPEED_MULTIPLIER); }
        else { bot.setMotorPowers(motorPowers[0], motorPowers[1], motorPowers[2], motorPowers[3], 1); }


    }
}
