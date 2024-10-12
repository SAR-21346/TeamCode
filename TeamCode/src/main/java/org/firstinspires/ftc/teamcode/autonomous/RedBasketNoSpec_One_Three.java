package org.firstinspires.ftc.teamcode.autonomous;

import static org.firstinspires.ftc.teamcode.RobotConstants.IntakeState.INTAKE_FLIP_IN;
import static org.firstinspires.ftc.teamcode.RobotConstants.IntakeState.INTAKE_FLIP_OUT;
import static org.firstinspires.ftc.teamcode.RobotConstants.IntakeState.INTAKE_INIT;
import static org.firstinspires.ftc.teamcode.RobotConstants.IntakeState.INTAKE_RELEASE;
import static org.firstinspires.ftc.teamcode.RobotConstants.IntakeState.INTAKE_RETRACT;
import static org.firstinspires.ftc.teamcode.RobotConstants.IntakeState.INTAKE_SAMPLE_IN;
import static org.firstinspires.ftc.teamcode.RobotConstants.IntakeState.INTAKE_SPIN;
import static org.firstinspires.ftc.teamcode.RobotConstants.IntakeState.INTAKE_STOP;
import static org.firstinspires.ftc.teamcode.RobotConstants.LiftState.BUCKET_TIP;
import static org.firstinspires.ftc.teamcode.RobotConstants.LiftState.LIFT_EXTEND_HIGH;
import static org.firstinspires.ftc.teamcode.RobotConstants.LiftState.LIFT_INIT;
import static org.firstinspires.ftc.teamcode.RobotConstants.LiftState.LIFT_RETRACT;
import static org.firstinspires.ftc.teamcode.RobotConstants.LiftState.LIFT_START;
import static org.firstinspires.ftc.teamcode.RobotConstants.LiftState.LIFT_STOP;
import static org.firstinspires.ftc.teamcode.autonomous.FieldConstants.ascentParkingBlue;
import static org.firstinspires.ftc.teamcode.autonomous.FieldConstants.redAllianceBasket;
import static org.firstinspires.ftc.teamcode.autonomous.FieldConstants.redAllianceBasketStart;
import static org.firstinspires.ftc.teamcode.autonomous.FieldConstants.redAllianceNeutralCenterSpike;
import static org.firstinspires.ftc.teamcode.autonomous.FieldConstants.redAllianceNeutralLeftSpike;
import static org.firstinspires.ftc.teamcode.autonomous.FieldConstants.redAllianceNeutralRightSpike;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.MecanumTrain;
import org.firstinspires.ftc.teamcode.RobotConstants.IntakeState;
import org.firstinspires.ftc.teamcode.RobotConstants.LiftState;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierCurve;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;
import org.firstinspires.ftc.teamcode.pedroPathing.util.Timer;

@Autonomous(name = "Red Basket No Specimen Preload + 3", group = "Red Basket", preselectTeleOp = "Red TeleOp")
public class RedBasketNoSpec_One_Three extends OpMode {

    private IntakeState intakeState;
    private LiftState liftState;
    private Timer intakeTimer, pathTimer, liftTimer;
    private ElapsedTime opmodeTimer;

    private MecanumTrain bot;

    private int pathState;

    Pose startPose = new Pose(
            redAllianceBasketStart.getX(),
            redAllianceBasketStart.getY(),
            redAllianceBasketStart.getHeading());

    Path preloadBasketScore, rightSampleCycle, rightSampleScore,
            centerSampleCycle, centerSampleScore, leftSampleCycle, leftSampleScore, parkPath;


    @Override
    public void init() {
        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        intakeTimer = new Timer();
        pathTimer = new Timer();
        liftTimer = new Timer();
        opmodeTimer = new ElapsedTime();

        bot = new MecanumTrain(hardwareMap, opmodeTimer);
        bot.follower.setStartingPose(redAllianceBasketStart);

        bot.setIntakeServo("off");
        bot.setIntakePivot("in");
        bot.setHorizontalExtension("in");
    }

    @Override
    public void start() {
        opmodeTimer.reset();
        buildPaths();
        setIntakeState(INTAKE_INIT);
        setLiftState(LIFT_INIT);
        setPathState(1);
    }
    @Override
    public void loop() {
        bot.follower.update();

        autonomousPathUpdate();
        intakeStateUpdate();
        outtakeStateUpdate();

        telemetry.addData("Path State", pathState);
        telemetry.addData("Intake State", intakeState);
        telemetry.addData("Lift State", liftState);
        telemetry.update();
    }

    private void buildPaths() {
        preloadBasketScore = new Path(new BezierLine(
                new Point(startPose),
                new Point(redAllianceBasket)
        ));
        preloadBasketScore.setLinearHeadingInterpolation(startPose.getHeading(), redAllianceBasket.getHeading());
        preloadBasketScore.setPathEndTimeoutConstraint(0);

        rightSampleCycle = new Path(new BezierLine(
                new Point(redAllianceBasket),
                new Point(redAllianceNeutralRightSpike.getX()+4, redAllianceNeutralRightSpike.getY()+11, Point.CARTESIAN)
        ));
        rightSampleCycle.setLinearHeadingInterpolation(Math.toRadians(+45), Math.toRadians(90));
        rightSampleCycle.setPathEndTimeoutConstraint(0);

        rightSampleScore = new Path(new BezierLine(
                new Point(redAllianceNeutralRightSpike.getX()+4, redAllianceNeutralRightSpike.getY()+11, Point.CARTESIAN),
                new Point(redAllianceBasket)));
        rightSampleScore.setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(+45));
        rightSampleScore.setPathEndTimeoutConstraint(0);

        centerSampleCycle = new Path(new BezierLine(
                new Point(redAllianceBasket),
                new Point(redAllianceNeutralCenterSpike.getX()+4, redAllianceNeutralCenterSpike.getY()+11, Point.CARTESIAN)
        ));
        centerSampleCycle.setLinearHeadingInterpolation(redAllianceBasket.getHeading(), Math.toRadians(90));
        centerSampleCycle.setPathEndTimeoutConstraint(0);

        centerSampleScore = new Path(new BezierLine(
                new Point(redAllianceNeutralCenterSpike.getX()+4, redAllianceNeutralCenterSpike.getY()+11, Point.CARTESIAN),
                new Point(redAllianceBasket)));
        centerSampleScore.setLinearHeadingInterpolation(Math.toRadians(90), redAllianceBasket.getHeading());
        centerSampleScore.setPathEndTimeoutConstraint(0);

        leftSampleCycle = new Path(new BezierCurve(
                new Point(redAllianceBasket),
                new Point(44.0, 117.0, Point.CARTESIAN),
                new Point(redAllianceNeutralLeftSpike.getX()+4, redAllianceNeutralLeftSpike.getY()+11, Point.CARTESIAN)
        ));
        leftSampleCycle.setLinearHeadingInterpolation(redAllianceBasket.getHeading(), Math.toRadians(90));
        leftSampleCycle.setPathEndTimeoutConstraint(0);

        leftSampleScore = new Path(new BezierLine(
                new Point(redAllianceNeutralLeftSpike.getX()+4, redAllianceNeutralLeftSpike.getY()+11, Point.CARTESIAN),
                new Point(redAllianceBasket)));
        leftSampleScore.setLinearHeadingInterpolation(Math.toRadians(90), redAllianceBasket.getHeading());
        leftSampleScore.setPathEndTimeoutConstraint(0);

        parkPath = new Path(new BezierLine(
                new Point(redAllianceBasket),
                new Point(ascentParkingBlue)
        ));
        parkPath.setLinearHeadingInterpolation(redAllianceBasket.getHeading(), ascentParkingBlue.getHeading());
        parkPath.setPathEndTimeoutConstraint(0);
    }

    private void autonomousPathUpdate() {
        switch(pathState) {
            case 1: // Path to the basket (preload)
                bot.follower.followPath(preloadBasketScore, true);
                setPathState(2);
                break;
            case 2: // Run the lift and outtake
                if (pathTimer.getElapsedTimeSeconds() > 1.5) {
                    setLiftState(LIFT_START);
                    if (liftTimer.getElapsedTimeSeconds() > 1.8) {
                        bot.setBucket("tip");
                        setPathState(3);
                    }
                }
                break;
            case 3: // Path to the right sample cycle
                bot.setBucket("flat");
                bot.follower.followPath(rightSampleCycle);
                setPathState(4);
                break;
            case 4:  // Flip out the intake at 95% through the path
                if (bot.follower.getCurrentTValue() > 0.95) {
                    setIntakeState(INTAKE_FLIP_OUT);
                    setPathState(5);
                }
                break;
            case 5: // Retract the intake and follow path to bucket
                if (intakeState == INTAKE_RETRACT) {
                    bot.follower.followPath(rightSampleScore, true);
                    setPathState(6);
                }
                break;
            case 6: // Run the lift and outtake
                if (pathTimer.getElapsedTimeSeconds() > 1.5) {
                    setLiftState(LIFT_START);
                    if (bot.verticalExtension.getCurrentPosition() >= 1200) {
                        bot.setBucket("tip");
                    }

                    if (liftState == LIFT_STOP) {
                        setPathState(7);
                    }
                }
                break;
            case 7: // Path to the center sample cycle
                bot.setBucket("flat");
                bot.follower.followPath(centerSampleCycle);
                setPathState(8);
                break;
            case 8: // Flip out the intake at 95% through the path
                if (bot.follower.getCurrentTValue() > 0.95) {
                    setIntakeState(INTAKE_FLIP_OUT);
                    setPathState(9);
                }
                break;
            case 9: // Retract the intake and follow path to bucket
                if (intakeState == INTAKE_RETRACT) {
                    bot.follower.followPath(centerSampleScore, true);
                    setPathState(10);
                }
                break;
            case 10: // Run the lift and outtake
                if (pathTimer.getElapsedTimeSeconds() > 1.5) {
                    setLiftState(LIFT_START);
                    if (bot.verticalExtension.getCurrentPosition() >= 1200) {
                        bot.setBucket("tip");
                    }

                    if (liftState == LIFT_STOP) {
                        setPathState(11);
                    }
                }
                break;
            case 11: // Path to the left sample cycle
                bot.setBucket("flat");
                bot.follower.followPath(leftSampleCycle);
                setPathState(12);
                break;
            case 12: // Flip out the intake at 95% through the path
                if (bot.follower.getCurrentTValue() > 0.95) {
                    setIntakeState(INTAKE_FLIP_OUT);
                    setPathState(13);
                }
                break;
            case 13: // Retract the intake and follow path to bucket
                if (intakeState == INTAKE_RETRACT) {
                    bot.follower.followPath(leftSampleScore, true);
                    setPathState(14);
                }
                break;
            case 14: // Run the lift and outtake
                if (pathTimer.getElapsedTimeSeconds() > 1.5) {
                    setLiftState(LIFT_START);
                    if (bot.verticalExtension.getCurrentPosition() >= 1200) {
                        bot.setBucket("tip");
                    }

                    if (liftState == LIFT_STOP) {
                        setPathState(15);
                    }
                }
                break;
            case 15: // Path to park
                bot.follower.followPath(parkPath);
                setPathState(16);
                break;
            case 16: // Stop the auto after parking
                if(!bot.follower.isBusy()) {
                    requestOpModeStop();
                }
                break;
        }
    }

    private void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
        autonomousPathUpdate();
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

    private void intakeStateUpdate () {
        switch (intakeState) {
            case INTAKE_INIT:
                break;
            case INTAKE_FLIP_OUT:
                bot.setIntakePivot("out");
                setIntakeState(INTAKE_SPIN);
                break;
            case INTAKE_SPIN:
                bot.setIntakeServo("forward");
                if (bot.sampleDetected()) {
                    setIntakeState(INTAKE_SAMPLE_IN);
                }
                break;
            case INTAKE_SAMPLE_IN:
                bot.setIntakeServo("off");
                setIntakeState(INTAKE_RETRACT);
                break;
            case INTAKE_RETRACT:
                bot.setHorizontalExtension("in");
                if (intakeTimer.getElapsedTimeSeconds() > 0.8) {
                    setIntakeState(INTAKE_FLIP_IN);
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
            case INTAKE_RELEASE:
                bot.setIntakePivot("in");
                bot.setIntakeServo("backward");
                setIntakeState(INTAKE_STOP);
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
                if (bot.verticalExtension.getCurrentPosition() >= 1200) {
                    setLiftState(BUCKET_TIP);
                }
                break;
            case BUCKET_TIP:
                bot.setBucket("tip");
                if (!bot.sampleInOuttake()) {
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
                break;
            case LIFT_RELEASE:
                break;

        }
    }

}
