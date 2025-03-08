package org.firstinspires.ftc.teamcode.teleop;

import static org.firstinspires.ftc.teamcode.RobotConstants.INTAKE_POWER_NEG;
import static org.firstinspires.ftc.teamcode.RobotConstants.INTAKE_POWER_POS;
import static org.firstinspires.ftc.teamcode.RobotConstants.IntakeState.DISTANCE_CHECK;
import static org.firstinspires.ftc.teamcode.RobotConstants.IntakeState.EXTEND;
import static org.firstinspires.ftc.teamcode.RobotConstants.IntakeState.INIT;
import static org.firstinspires.ftc.teamcode.RobotConstants.IntakeState.INTAKE_ACCEPT;
import static org.firstinspires.ftc.teamcode.RobotConstants.IntakeState.INTAKE_ENABLE;
import static org.firstinspires.ftc.teamcode.RobotConstants.IntakeState.INTAKE_REJECT;
import static org.firstinspires.ftc.teamcode.RobotConstants.IntakeState.INTAKE_REJECT_BYPASS;
import static org.firstinspires.ftc.teamcode.RobotConstants.IntakeState.PIVOT_DOWN;
import static org.firstinspires.ftc.teamcode.RobotConstants.IntakeState.PIVOT_DOWN_BYPASS;
import static org.firstinspires.ftc.teamcode.RobotConstants.IntakeState.PIVOT_UP;
import static org.firstinspires.ftc.teamcode.RobotConstants.IntakeState.STOP;
import static org.firstinspires.ftc.teamcode.RobotConstants.LIFT_HIGH_BUCKET;
import static org.firstinspires.ftc.teamcode.RobotConstants.OuttakeState.EXTEND_HIGH_BUCKET;
import static org.firstinspires.ftc.teamcode.RobotConstants.OuttakeState.EXTEND_HIGH_SPEC;
import static org.firstinspires.ftc.teamcode.RobotConstants.OuttakeState.INTAKE_GRAB;
import static org.firstinspires.ftc.teamcode.RobotConstants.OuttakeState.RETRACT;
import static org.firstinspires.ftc.teamcode.RobotConstants.OuttakeState.SCORE_HIGH_BUCKET;
import static org.firstinspires.ftc.teamcode.RobotConstants.OuttakeState.SCORE_HIGH_SPEC;
import static org.firstinspires.ftc.teamcode.RobotConstants.OuttakeState.SPEC_PICKUP;
import static org.firstinspires.ftc.teamcode.RobotConstants.OuttakeState.START;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.MecanumTrain;
import org.firstinspires.ftc.teamcode.RobotConstants.IntakeState;
import org.firstinspires.ftc.teamcode.RobotConstants.OuttakeState;

@TeleOp(name = "Blue Drive")
public class BlueTeleOp extends OpMode {
    MecanumTrain bot;

    IntakeState intakeState;
    OuttakeState outtakeState;
    Timer intakeTimer, outtakeTimer;

    double SPEED_MULTIPLIER = 1;

    @Override
    public void init() {
        bot = new MecanumTrain(hardwareMap);

        intakeTimer = new Timer();
        outtakeTimer = new Timer();

        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        setIntakeState(INIT);
        setOuttakeState(OuttakeState.INIT);
    }

    @Override
    public void start() {
        bot.resetLift();
    }

    @Override
    public void loop() {
        double y = -gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x;
        double rx = gamepad1.right_stick_x;

        double[] powers = bot.calculateMotorPowers(y,x,rx);

        if (gamepad1.left_bumper) {
            SPEED_MULTIPLIER = 0.50;
        } else {
            double currentPosition = bot.liftR.getCurrentPosition();
            currentPosition = Math.min(currentPosition, 3400);
            SPEED_MULTIPLIER = 1 - (currentPosition / 3400);

            SPEED_MULTIPLIER = Math.max(0.15, Math.min(1, SPEED_MULTIPLIER));
        }


        bot.setMotorPowers(powers[0], powers[1], powers[2], powers[3], SPEED_MULTIPLIER);

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

        if (gamepad2.dpad_up) {
            setOuttakeState(START);
        }

        if (gamepad2.left_stick_button) {
            setOuttakeState(RETRACT);
        }

        if (gamepad2.share) {
            setIntakeState(INTAKE_REJECT_BYPASS);
        }


        bot.encoderUpdate();
        bot.distSensorUpdate();
        bot.updateLift();
        intakeStateUpdate();
        outtakeStateUpdate();

        telemetry.addData("current intake state", intakeState);
        telemetry.addData("liftPos", bot.liftR.getCurrentPosition());
        telemetry.addData("current outtake state", outtakeState);
    }

    private void intakeStateUpdate() {
        switch (intakeState) {
            case INIT:
                bot.intake_retract();
                bot.intake_pivot_up();
                break;
            case DISTANCE_CHECK:
                // Check distance
                if ((bot.leftDistVal + bot.rightDistVal) / 2 > 12) { // replace "true" with distance check
                    setIntakeState(EXTEND);
                }
                break;
            case EXTEND:
                // Extend horizontal slides
                bot.intake_extend(5);
                setIntakeState(PIVOT_DOWN);
                break;
            case PIVOT_DOWN:
                if(intakeTimer.getElapsedTimeSeconds() > 0.3) { // replace "true" with slide limit check
                    bot.outtake_clearance();
                    bot.intake_pivot_down();
                    setIntakeState(INTAKE_ENABLE);
                }
                break;
            case PIVOT_DOWN_BYPASS:
                bot.intake_pivot_down();
                bot.outtake_clearance();
                setIntakeState(INTAKE_ENABLE);
                break;
            case INTAKE_ENABLE:
                // Pick up sample
                bot.intake.setPower(INTAKE_POWER_POS);
                bot.distSensorUpdate();
                if (bot.intakeWheelDist < 25 || bot.intakeWallDetect()) {
                    if (bot.intakeWheelDetect() == 2 || bot.intakeWheelDetect() == 3) { // 2 for blue, 3 for yellow
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
            case INTAKE_REJECT_BYPASS:
                bot.intake_pivot_down();
                bot.intake.setPower(INTAKE_POWER_NEG);
                if (intakeTimer.getElapsedTimeSeconds() > 0.4) {
                    setIntakeState(STOP);
                }
                break;
            case INTAKE_ACCEPT:
                bot.distSensorUpdate();
                if (bot.intakeWallDetect()) {
                    bot.intake.setPower(0);
                    setIntakeState(PIVOT_UP);
                }
            case PIVOT_UP:
                // pivot intake up
                bot.intake_pivot_up();
                // retract slides
                bot.intake_retract();
                bot.intake.setPower(0);
                bot.claw_close();
                if (intakeTimer.getElapsedTimeSeconds() > 0.8) {
                    bot.claw_open();
                    bot.outtake_flat();
                    setIntakeState(STOP);
                }
                break;
            case STOP:
                // pivot intake up
                bot.intake_pivot_up();
                // retract slides
                bot.intake_retract();
                bot.intake.setPower(0);
                break;
        }
    }

    private void outtakeStateUpdate() {
        switch (outtakeState) {
            case INIT:
                // make sure outtake is reset to right pos
                bot.resetLift();
                bot.claw_open();
                break;
            case START:
                bot.outtake_flat();
                setOuttakeState(INTAKE_GRAB);
                break;
            case INTAKE_GRAB:
                if (gamepad2.dpad_left) {
                    setOuttakeState(SPEC_PICKUP);
                }
                if(bot.intakeWallDetect()) { // Checks if something is in the intake
                    bot.claw_close();
                    setOuttakeState(EXTEND_HIGH_BUCKET);
                }
                break;
            case EXTEND_HIGH_BUCKET:
                // Extend it to top basket
                bot.extend_high_bucket();
                if (bot.liftR.getCurrentPosition() >= LIFT_HIGH_BUCKET-50) {
                    setOuttakeState(SCORE_HIGH_BUCKET);
                }
                break;
            case SCORE_HIGH_BUCKET:
                bot.outtake_score_bucket();
                if (gamepad2.left_bumper) { // replace with encoder value of outtake servo
                    bot.claw_open();
                    setOuttakeState(RETRACT);
                }
                break;
            case SPEC_PICKUP:
                // set outtake to spec pos
                bot.outtake_spec();
                // wait for outtake encoder pos
                if (gamepad2.right_bumper) {
                    bot.claw_close();
                    setOuttakeState(EXTEND_HIGH_SPEC);
                }
                break;
            case EXTEND_HIGH_SPEC:
                if (outtakeTimer.getElapsedTimeSeconds() > 0.8) {
                    // Score it to top rung
                    bot.extend_high_spec();
                    bot.outtake_score_spec();
                    if (gamepad2.left_bumper) { // replace with encoder value of outtake servo
                        setOuttakeState(SCORE_HIGH_SPEC);
                    }
                }
                break;
            case SCORE_HIGH_SPEC:
                // Code for attaching spec to rung
                bot.outtake_score_spec_2();
                if (outtakeTimer.getElapsedTimeSeconds() > 0.6) {
                    bot.claw_open();
                    setOuttakeState(RETRACT);
                }
                break;
            case RETRACT:
                // wait for claw
                if (outtakeTimer.getElapsedTimeSeconds() > 0.5) {
                    bot.outtake_flat();
                    setOuttakeState(OuttakeState.STOP);
                }
            case STOP:
                if (bot.liftR.getCurrentPosition() > 200) {
                    bot.retractLift();
                } else {
                    bot.resetLift();
                }
                bot.claw_open();
                break;

        }
    }

    private void setIntakeState(IntakeState iState) {
        intakeState = iState;
        intakeTimer.resetTimer();
        intakeStateUpdate();
    }

    private void setOuttakeState (OuttakeState oState) {
        outtakeState = oState;
        outtakeTimer.resetTimer();
        outtakeStateUpdate();
    }
}
