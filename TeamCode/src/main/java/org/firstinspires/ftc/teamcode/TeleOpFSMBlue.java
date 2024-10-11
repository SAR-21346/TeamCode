package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.RobotConstants.IntakeState.INTAKE_EXTEND;
import static org.firstinspires.ftc.teamcode.RobotConstants.IntakeState.INTAKE_FLIP_IN;
import static org.firstinspires.ftc.teamcode.RobotConstants.IntakeState.INTAKE_FLIP_OUT;
import static org.firstinspires.ftc.teamcode.RobotConstants.IntakeState.INTAKE_INIT;
import static org.firstinspires.ftc.teamcode.RobotConstants.IntakeState.INTAKE_RELEASE;
import static org.firstinspires.ftc.teamcode.RobotConstants.IntakeState.INTAKE_RETRACT;
import static org.firstinspires.ftc.teamcode.RobotConstants.IntakeState.INTAKE_SAMPLE_IN;
import static org.firstinspires.ftc.teamcode.RobotConstants.IntakeState.INTAKE_SPIN;
import static org.firstinspires.ftc.teamcode.RobotConstants.IntakeState.INTAKE_START;
import static org.firstinspires.ftc.teamcode.RobotConstants.IntakeState.INTAKE_STOP;
import static org.firstinspires.ftc.teamcode.RobotConstants.LiftState.BUCKET_TIP;
import static org.firstinspires.ftc.teamcode.RobotConstants.LiftState.LIFT_EXTEND_HIGH;
import static org.firstinspires.ftc.teamcode.RobotConstants.LiftState.LIFT_RETRACT;
import static org.firstinspires.ftc.teamcode.RobotConstants.LiftState.LIFT_SAMPLE_RELEASED;
import static org.firstinspires.ftc.teamcode.RobotConstants.LiftState.LIFT_START;
import static org.firstinspires.ftc.teamcode.RobotConstants.LiftState.LIFT_STOP;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.RobotConstants.IntakeState;
import org.firstinspires.ftc.teamcode.RobotConstants.LiftState;
import org.firstinspires.ftc.teamcode.pedroPathing.util.Timer;


@TeleOp(name = "Blue TeleOp", group = "TeleOp")
public class TeleOpFSMBlue extends OpMode {
    MecanumTrain bot;

    private IntakeState intakeState;
    private Timer intakeTimer;

    private LiftState liftState;
    private Timer liftTimer;

    private ElapsedTime opmodeTimer;
    private final double SPEED_MULTIPLIER = 0.50;

    int target = 0;



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
        opmodeTimer.reset();

        setIntakeState(INTAKE_INIT);
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

        if (gamepad2.a) {
            bot.liftRetract();
        }

        if (gamepad2.b) {
            bot.liftExtend_highBucket();
        }

        // -------------- LIFT ---------------- (JUST FOR TEST)
        if(gamepad2.right_trigger > 0){
            float decreaseFlip = gamepad2.right_trigger;
            target += (int) (decreaseFlip*5);
        }

        if (gamepad2.left_trigger > 0) {
            float decreaseFlip = gamepad2.left_trigger;
            target -= (int) (decreaseFlip*5);
        }

        if(gamepad2.dpad_up) {
            bot.setBucket("tip");
        }

        // -------------- DRIVE ----------------
        double axial = gamepad1.left_stick_y;
        double lateral = gamepad1.left_stick_x;
        double yaw = -gamepad1.right_stick_x;

        // calculate motor powers
        double[] motorPowers = bot.calculateMotorPowers(axial, lateral, yaw);
//        bot.runLift(target);
        if (gamepad1.right_bumper) { bot.setMotorPowers(motorPowers[0], motorPowers[1], motorPowers[2], motorPowers[3], SPEED_MULTIPLIER); }
        else { bot.setMotorPowers(motorPowers[0], motorPowers[1], motorPowers[2], motorPowers[3], 1); }


        // -------------- TELEMETRY ---------------
        telemetry.addData("distLeft", bot.leftFrontDist.getDistance(DistanceUnit.CM));
        telemetry.addData("distRight", bot.rightFrontDist.getDistance(DistanceUnit.CM));
        telemetry.addData("lift encoder", bot.verticalExtension.getCurrentPosition());
        telemetry.addData("horizontalLimit", bot.horizontalLimit.isPressed());
        telemetry.addData("leftEnc", bot.leftFrontDrive.getCurrentPosition());
        telemetry.addData("rightEnc", bot.leftBackDrive.getCurrentPosition());
        telemetry.addData("strafeEnc", bot.rightFrontDrive.getCurrentPosition());
        telemetry.addData("current intake state", intakeState);

        telemetry.update();
    }



    private boolean intakeDistCheck = false;
    private void intakeStateUpdate () {
        switch (intakeState) {
            case INTAKE_INIT:
                bot.setIntakeServo("off");
                bot.setIntakePivot("in");
                bot.setHorizontalExtension("in");
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
                bot.setIntakePivot("mid");
                if (bot.horizontalLimit.isPressed()) {
                    intakeTimer.resetTimer();
                    bot.setIntakePivot("in");
                    if (intakeTimer.getElapsedTimeSeconds() > 10) {
                        bot.setIntakeServo("backward");
                    }
                }
                setIntakeState(INTAKE_RELEASE);
                break;
            case INTAKE_SPIN:
                bot.setIntakeServo("forward");
                if (bot.sampleDetected()) {
                    setIntakeState(INTAKE_SAMPLE_IN);
                }
                break;
            case INTAKE_SAMPLE_IN:
                bot.setIntakeServo("off");
                int r = bot.intakeColor.red(), g = bot.intakeColor.green(), b = bot.intakeColor.blue();
                int maxValue = Math.max(r, Math.max(g, b));
                if (bot.sampleDetected() && maxValue == r) {
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

    private void outtakeStateUpdate() {
        switch(liftState) {
            case LIFT_INIT:
                bot.resetLift();
                setLiftState(LIFT_START);
                break;
            case LIFT_START:
                bot.liftRetract();
                setLiftState(LIFT_EXTEND_HIGH);
                break;
            case LIFT_EXTEND_LOW:
//                if(sampleDetectedInBucket()) {
//                    bot.liftExtend_lowBucket();
//                }
                break;
            case LIFT_EXTEND_HIGH:
//                if(sampleDetectedInBucket()) {
//                    bot.liftExtend_highBucket();
//                }
                setLiftState(BUCKET_TIP);
                break;
            case BUCKET_TIP:
                bot.setBucket("tip");
                setLiftState(LIFT_SAMPLE_RELEASED);
                break;
            case LIFT_SAMPLE_RELEASED:
//                if(!sampleDetectedInBucket()) {
//                    bot.setBucket("flat");
//                }
                setLiftState(LIFT_RETRACT);
                break;
            case LIFT_RETRACT:
                bot.resetLift();
                setLiftState(LIFT_STOP);
                break;
            case LIFT_STOP:
                bot.stopLift();
                break;
            case LIFT_RELEASE:
                break;

        }
    }



    private void setIntakeState (IntakeState iState) {
        intakeState = iState;
        intakeTimer.resetTimer();
        intakeStateUpdate();
    }

    private void setLiftState (LiftState lState) {
        liftState = lState;
        liftTimer.resetTimer();
        outtakeStateUpdate();
    }
}
