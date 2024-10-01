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
import static org.firstinspires.ftc.teamcode.RobotConstants.IntakeState.INTAKE_STOP;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.RobotConstants.IntakeState;
import org.firstinspires.ftc.teamcode.pedroPathing.util.Timer;

@TeleOp(name = "TeleOpFSMBlue")
public class TeleOpFSMBlue extends OpMode {
    MecanumTrain bot;

    private IntakeState intakeState;
    private Timer intakeTimer;
    private ElapsedTime opmodeTimer;
    private final double SPEED_MULTIPLIER = 0.50;

    @Override
    public void init() {
        intakeTimer = new Timer();
        opmodeTimer = new ElapsedTime(); 

        opmodeTimer.reset();

        bot = new MecanumTrain(hardwareMap, opmodeTimer);
        bot.verticalExtension.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
    }

    @Override
    public void start() {
        bot.setIntakeServo("off");
        bot.setIntakePivot("in");
        bot.setHorizontalExtension("in");

        opmodeTimer.reset();

        intakeState = INTAKE_START;
    }

    @Override
    public void loop() {
        // -------------- INTAKE ----------------
         intakeStateUpdate();

        if (gamepad2.x) {
            setIntakeState(INTAKE_START);
            intakeDistCheck = true;
        }

        if (gamepad2.dpad_down) {
            setIntakeState(INTAKE_FLIP_OUT);
        }

        if (gamepad2.y) {
            setIntakeState(INTAKE_STOP);
        }

        // -------------- DRIVE ----------------


        double axial = gamepad1.left_stick_y;
        double lateral = gamepad1.left_stick_x;
        double yaw = -gamepad1.right_stick_x;

        // calculate motor powers
        double[] motorPowers = bot.calculateMotorPowers(axial, lateral, yaw);

        if (gamepad1.right_bumper) { bot.setMotorPowers(motorPowers[0], motorPowers[1], motorPowers[2], motorPowers[3], SPEED_MULTIPLIER); }
        else { bot.setMotorPowers(motorPowers[0], motorPowers[1], motorPowers[2], motorPowers[3], 1); }


        // -------------- TELEMETRY ---------------
        telemetry.addData("distLeft", bot.leftFrontDist.getDistance(DistanceUnit.CM));
        telemetry.addData("distRight", bot.rightFrontDist.getDistance(DistanceUnit.CM));
        telemetry.addData("intakeColorRed", bot.intakeColor.red());
        telemetry.addData("intakeColorBlue", bot.intakeColor.blue());
        telemetry.addData("intakeColorGreen", bot.intakeColor.green());
        telemetry.addData("horizontalLimit", bot.horizontalLimit.isPressed());
        telemetry.addData("current intake state", intakeState);

        telemetry.update();
    }



    private boolean intakeDistCheck = false;
    private void intakeStateUpdate () {
        switch (intakeState) {
            case INTAKE_START:
                if (intakeDistCheck && (bot.leftFrontDist.getDistance(DistanceUnit.CM) + bot.rightFrontDist.getDistance(DistanceUnit.CM)) / 2 < 15) {
                    setIntakeState(INTAKE_EXTEND);
                    intakeDistCheck = false;
                }
                break;
            case INTAKE_EXTEND:
                bot.setHorizontalExtension("out");
                if (intakeTimer.getElapsedTimeSeconds() > 0.7) {
                    setIntakeState(INTAKE_FLIP_OUT);
                }
                break;
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
                    if (intakeTimer.getElapsedTimeSeconds() > 1) {
                        bot.setIntakeServo("backward");
                    }
                }
                setIntakeState(INTAKE_RELEASE);
                break;
            case INTAKE_SPIN:
                bot.setIntakeServo("forward");
                if(sampleDetected()) {
                    setIntakeState(INTAKE_SAMPLE_IN);
                }
                break;
            case INTAKE_SAMPLE_IN:
                bot.setIntakeServo("off");
                int r = bot.intakeColor.red(), g = bot.intakeColor.green(), b = bot.intakeColor.blue();
                int maxValue = Math.max(r, Math.max(g, b));
                if (sampleDetected() && maxValue == r) {
                    bot.setIntakeServo("backward");
                }
                setIntakeState(INTAKE_RETRACT);
                break;
            case INTAKE_SAMPLE_OUT:
                bot.setIntakeServo("forward");
                if (sampleDetected()) {
                    setIntakeState(intakeColorCheck());
                }
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

    private IntakeState intakeColorCheck() {
        int r = bot.intakeColor.red();
        int g = bot.intakeColor.green();
        int b = bot.intakeColor.blue();

        int maxValue = Math.max(r, Math.max(g, b));

        if (maxValue == r) {
            return INTAKE_SAMPLE_OUT;
        }
        if (maxValue == b || (r+g)/2 > b) {
            return INTAKE_RETRACT;
        }

        return INTAKE_SPIN;
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

    private void setIntakeState (IntakeState iState) {
        intakeState = iState;
        intakeTimer.resetTimer();
        intakeStateUpdate();
    }
}
