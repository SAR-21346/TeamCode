package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.RobotConstants.AutoMovementState.INTAKE;
import static org.firstinspires.ftc.teamcode.RobotConstants.AutoMovementState.MOVE_BUCKET;
import static org.firstinspires.ftc.teamcode.RobotConstants.AutoMovementState.MOVE_BUCK_SUB;
import static org.firstinspires.ftc.teamcode.RobotConstants.AutoMovementState.MOVE_INIT;
import static org.firstinspires.ftc.teamcode.RobotConstants.AutoMovementState.MOVE_STOP;
import static org.firstinspires.ftc.teamcode.RobotConstants.AutoMovementState.MOVE_SUB;
import static org.firstinspires.ftc.teamcode.RobotConstants.AutoMovementState.OUTAKE;
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

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.RobotConstants.IntakeState;
import org.firstinspires.ftc.teamcode.RobotConstants.LiftState;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierCurve;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathBuilder;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;
import org.firstinspires.ftc.teamcode.pedroPathing.util.Timer;

import static org.firstinspires.ftc.teamcode.RobotConstants.IntakeState.INTAKE_INIT;
import static org.firstinspires.ftc.teamcode.RobotConstants.IntakeState.INTAKE_FLIP_IN;
import static org.firstinspires.ftc.teamcode.RobotConstants.IntakeState.INTAKE_FLIP_OUT;
import static org.firstinspires.ftc.teamcode.RobotConstants.IntakeState.INTAKE_INIT;
import static org.firstinspires.ftc.teamcode.RobotConstants.IntakeState.INTAKE_RELEASE;
import static org.firstinspires.ftc.teamcode.RobotConstants.IntakeState.INTAKE_RETRACT;
import static org.firstinspires.ftc.teamcode.RobotConstants.IntakeState.INTAKE_SAMPLE_IN;
import static org.firstinspires.ftc.teamcode.RobotConstants.IntakeState.INTAKE_SPIN;
import static org.firstinspires.ftc.teamcode.RobotConstants.IntakeState.INTAKE_START;
import static org.firstinspires.ftc.teamcode.RobotConstants.IntakeState.INTAKE_STOP;
import static org.firstinspires.ftc.teamcode.RobotConstants.LiftState.LIFT_INIT;

@Autonomous(name = "Blue Auto", group = "AUTO")
public class AutoFSMBlue extends OpMode {
    MecanumTrain bot;

    private IntakeState intakeState;
    private Timer intakeTimer;

    private LiftState liftState;
    private Timer liftTimer;

    private RobotConstants.AutoMovementState moveState;
    

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

    }

    @Override
    public void loop() {
        setMoveState(MOVE_INIT);
        movementStateUpdate();


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

    private void movementStateUpdate () {
        PathBuilder builder = new PathBuilder();

        switch(moveState) {
            case MOVE_INIT:
                // DK what to add here
            case MOVE_SUB:
                builder
                .addPath(
                    // Line 1
                    new BezierCurve(
                    new Point(9.757, 84.983, Point.CARTESIAN),
                    new Point(22.011, 77.436, Point.CARTESIAN),
                    new Point(37.392, 78.232, Point.CARTESIAN)
                    )
                )
                .setLinearHeadingInterpolation(bot.follower.getHeadingOffset(), bot.follower.getHeadingOffset());
                setMoveState(INTAKE);
            case INTAKE:
                // Not sure if this works - but let's just pray
                setIntakeState(INTAKE_INIT);
                intakeStateUpdate();
                setMoveState(MOVE_BUCKET);
            case MOVE_BUCKET:
                builder
                .addPath(
                    new BezierCurve(
                    new Point(37.392, 78.232, Point.CARTESIAN),
                    new Point(27.580, 118.807, Point.CARTESIAN),
                    new Point(14.586, 130.475, Point.CARTESIAN)
                    )
                )
                .setLinearHeadingInterpolation(bot.follower.getHeadingOffset(), bot.follower.getHeadingOffset()) // TODO: FIX THIS - THESE are random headings
                .setReversed(true);
                setMoveState(OUTAKE);
            case OUTAKE:
                setLiftState(LIFT_INIT);
                outtakeStateUpdate();
                setMoveState(MOVE_BUCK_SUB);
            case MOVE_BUCK_SUB:
                builder
                .addPath(
                    // Line 3
                    new BezierCurve(
                    new Point(14.586, 130.475, Point.CARTESIAN),
                    new Point(15.381, 78.497, Point.CARTESIAN),
                    new Point(37.392, 78.232, Point.CARTESIAN)
                    )
                )
                .setLinearHeadingInterpolation(bot.follower.getHeadingOffset(), bot.follower.getHeadingOffset());
                setMoveState(INTAKE);
            case MOVE_STOP:
                // reset everything
        }
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
                    if (intakeTimer.getElapsedTimeSeconds() > 4) {
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
                break;
            case LIFT_START:
                bot.liftRetract();
                break;
            case LIFT_EXTEND_LOW:
                bot.liftExtend_lowBucket();
                break;
            case LIFT_EXTEND_HIGH:

                break;
            case BUCKET_TIP:
                break;
            case LIFT_SAMPLE_RELEASED:
                break;
            case LIFT_RETRACT:
                break;
            case LIFT_STOP:
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

    private void setMoveState (RobotConstants.AutoMovementState mState) {
        moveState = mState;
        movementStateUpdate();
    }
}
