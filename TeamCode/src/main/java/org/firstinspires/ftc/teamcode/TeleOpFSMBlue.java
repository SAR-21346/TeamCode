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
import static org.firstinspires.ftc.teamcode.RobotConstants.LiftState.LIFT_INIT;
import static org.firstinspires.ftc.teamcode.RobotConstants.LiftState.LIFT_RETRACT;
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

    private IntakeState intakeState; // Intake State - changes based on the state machine
    private Timer intakeTimer, liftTimer; // Timer for intake and lift states

    private LiftState liftState; // Lift State - changes based on the state machine

    private ElapsedTime opmodeTimer; // Timer for the opmode
    private final double SPEED_MULTIPLIER = 0.50; // Speed multiplier for drivers (RB)

    private boolean manual = false;

    private int target = 0;

    @Override
    public void init() {
        // Initialize timers
        intakeTimer = new Timer();
        liftTimer = new Timer();
        opmodeTimer = new ElapsedTime(); 

        opmodeTimer.reset(); // Reset the opmode timer

        bot = new MecanumTrain(hardwareMap, opmodeTimer); // Initialize the robot object
        bot.verticalExtension.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); // Reset the lift encoder

        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        bot.follower.startTeleopDrive();
    }

    @Override
    public void start() {
        opmodeTimer.reset(); // Reset the opmode timer for the start of the opmode

        setIntakeState(INTAKE_INIT); // Set the intake state to INIT
        setLiftState(LIFT_INIT); // Set the lift state to INIT
    }

    @Override
    public void loop() {
        if (gamepad2.left_stick_button && gamepad2.right_stick_button) {
            manual = !manual; // toggle manual mode
        }

        if (manual) {
            if (gamepad2.dpad_down) {
                bot.setIntakePivot("out");
            }

            if (gamepad2.dpad_left) {
                bot.setIntakeServo("forward");
            }

            if (gamepad2.dpad_right) {
                bot.setIntakeServo("backward");
            }

            if (gamepad2.touchpad) {
                bot.setIntakeServo("off");
            }

            if (gamepad2.dpad_up) {
                bot.setIntakePivot("in");
            }

            if (gamepad2.left_bumper) {
                bot.setBucket("flat");
            }

            if (gamepad2.right_bumper) {
                bot.setBucket("tip");
            }

            if(gamepad2.left_trigger > 0){
                float decreaseHeight = gamepad2.left_trigger;
                target -= decreaseHeight*15;
            }

            if ((gamepad2.right_trigger > 0) && target <= 4000) {
                float increaseFlip = gamepad2.right_trigger;
                target += increaseFlip*15;
            }

            bot.runLift(target);

            if (gamepad2.square && gamepad2.circle) {
                bot.resetLift();
            }
        } else {
            intakeStateUpdate(); // Update the intake state

            if (gamepad2.x) {
                setIntakeState(INTAKE_START); // Start the intake state machine
                intakeDistCheck = true; // Set the distance check to true
            }

            if (gamepad2.dpad_down) {
                setIntakeState(INTAKE_FLIP_OUT);
            }

            if (gamepad2.y) {
                setIntakeState(INTAKE_STOP);
                setLiftState(LIFT_RETRACT);
            }

            if (gamepad2.b) {
                setLiftState(LIFT_START); // Start the lift state machine
            }

            outtakeStateUpdate(); // Update the lift state
        }

        // -------------- DRIVE ----------------
        double axial = -gamepad1.left_stick_y; // Get the axial value from the left stick y
        double lateral = -gamepad1.left_stick_x; // Get the lateral value from the left stick x
        double yaw = -gamepad1.right_stick_x; // Get the yaw value from the right stick x

        // calculate motor powers
        //double[] motorPowers = bot.calculateMotorPowers(axial, lateral, yaw);
        //if (gamepad1.right_bumper) { bot.setMotorPowers(motorPowers[0], motorPowers[1], motorPowers[2], motorPowers[3], SPEED_MULTIPLIER); }
        //else { bot.setMotorPowers(motorPowers[0], motorPowers[1], motorPowers[2], motorPowers[3], 1); }

        if (gamepad1.left_bumper) {
            bot.follower.setTeleOpMovementVectors(
                    axial * SPEED_MULTIPLIER,
                    lateral * SPEED_MULTIPLIER,
                    yaw * SPEED_MULTIPLIER
            );
        } else {
            bot.follower.setTeleOpMovementVectors(axial, lateral, yaw);
        }
        bot.follower.update();

        // -------------- TELEMETRY ---------------
        telemetry.addData("Manual", manual);
        telemetry.addData("distLeft", bot.leftFrontDist.getDistance(DistanceUnit.CM));
        telemetry.addData("distRight", bot.rightFrontDist.getDistance(DistanceUnit.CM));
        telemetry.addData("distBack", bot.backDist.getDistance(DistanceUnit.INCH));
        telemetry.addData("target", target);
        telemetry.addData("lift encoder", bot.verticalExtension.getCurrentPosition());
        telemetry.addData("horizontalLimit", bot.horizontalLimit.isPressed());
        telemetry.addData("leftEnc", bot.leftFrontDrive.getCurrentPosition());
        telemetry.addData("rightEnc", bot.leftBackDrive.getCurrentPosition());
        telemetry.addData("strafeEnc", bot.rightFrontDrive.getCurrentPosition());
        telemetry.addData("current intake state", intakeState);
        telemetry.addData("current lift state", liftState);

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
                if (intakeTimer.getElapsedTimeSeconds() > 0.3) {
                    setIntakeState(INTAKE_FLIP_OUT);
                }
                break;
            case INTAKE_FLIP_OUT:
                bot.setIntakePivot("out");
                if (intakeTimer.getElapsedTimeSeconds() > 0.4) {
                    setIntakeState(INTAKE_SPIN);
                }
                break;
            case INTAKE_SPIN:
                bot.setIntakeServo("forward");
                if (bot.sampleDetected()) {
                    setIntakeState(INTAKE_SAMPLE_IN);
                }
                break;
            case INTAKE_SAMPLE_IN:
                bot.setIntakeServo("off");
//                int r = bot.intakeColor.red(), g = bot.intakeColor.green(), b = bot.intakeColor.blue();
//                int maxValue = Math.max(r, Math.max(g, b));
//                if (bot.sampleDetected() && maxValue == r) {
//                    bot.setIntakeServo("backward");
//                    if (!bot.sampleDetected()){
//                        setIntakeState(INTAKE_SPIN);
//                    }
//                }
                setIntakeState(INTAKE_RETRACT);
                break;
            case INTAKE_FLIP_IN:
                bot.setIntakePivot("in");
                if (intakeTimer.getElapsedTimeSeconds() > 0.4) {
                    setIntakeState(INTAKE_RELEASE);
                }
                break;
            case INTAKE_RETRACT:
                bot.setHorizontalExtension("in");
                if (intakeTimer.getElapsedTimeSeconds() > 0.6) {
                    setIntakeState(INTAKE_FLIP_IN);
                }
                break;
            case INTAKE_RELEASE:
                if(intakeTimer.getElapsedTimeSeconds() > 0.5) {
                    bot.setIntakeServo("backward");
                    if(bot.sampleInOuttake()) {
                        setIntakeState(INTAKE_STOP);
                    }
                }
                break;
            case INTAKE_STOP:
                if(intakeTimer.getElapsedTimeSeconds() > 0.4) {
                    bot.setIntakeServo("off");
                    bot.setIntakePivot("in");
                    bot.setHorizontalExtension("in");
                }
                break;
        }
    }

    private void outtakeStateUpdate() {
        switch(liftState) {
            case LIFT_INIT:
                bot.resetLift();
                bot.setBucket("flat");
                break;
            case LIFT_START:
                bot.liftRetract();
                if (bot.sampleInOuttake()) {
                    setLiftState(LIFT_EXTEND_HIGH);
                }
                break;
            case LIFT_EXTEND_LOW:
                bot.liftExtend_lowBucket();
                if (bot.verticalExtension.getCurrentPosition() >= 450) {
                    setLiftState(BUCKET_TIP);
                }
                break;
            case LIFT_EXTEND_HIGH:
                bot.liftExtend_highBucket();
                if (gamepad2.dpad_up) {
                    setLiftState(BUCKET_TIP);
                }
                break;
            case BUCKET_TIP:
                bot.setBucket("tip");
                if (!bot.sampleInOuttake() && liftTimer.getElapsedTimeSeconds() > 0.5) {
                    setLiftState(LIFT_RETRACT);
                }
                break;
            case LIFT_RETRACT:
                bot.setBucket("flat");
                bot.liftRetract();
                if (bot.verticalExtension.getCurrentPosition() <= 0) {
                    setLiftState(LIFT_STOP);
                }
                break;
            case LIFT_STOP:
                bot.verticalExtension.setPower(0);
                bot.setBucket("flat");
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
