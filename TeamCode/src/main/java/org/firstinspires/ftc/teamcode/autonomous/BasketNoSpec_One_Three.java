package org.firstinspires.ftc.teamcode.autonomous;

import static org.firstinspires.ftc.teamcode.RobotConstants.IntakeState.INTAKE_FLIP_IN;
import static org.firstinspires.ftc.teamcode.RobotConstants.IntakeState.INTAKE_FLIP_OUT;
import static org.firstinspires.ftc.teamcode.RobotConstants.IntakeState.INTAKE_INIT;
import static org.firstinspires.ftc.teamcode.RobotConstants.IntakeState.INTAKE_RETRACT;
import static org.firstinspires.ftc.teamcode.RobotConstants.IntakeState.INTAKE_SAMPLE_IN;
import static org.firstinspires.ftc.teamcode.RobotConstants.IntakeState.INTAKE_SAMPLE_OUT;
import static org.firstinspires.ftc.teamcode.RobotConstants.IntakeState.INTAKE_SPIN;
import static org.firstinspires.ftc.teamcode.RobotConstants.IntakeState.INTAKE_STOP;
import static org.firstinspires.ftc.teamcode.RobotConstants.LiftState.BUCKET_TIP;
import static org.firstinspires.ftc.teamcode.RobotConstants.LiftState.LIFT_EXTEND_HIGH;
import static org.firstinspires.ftc.teamcode.RobotConstants.LiftState.LIFT_EXTEND_LOW;
import static org.firstinspires.ftc.teamcode.RobotConstants.LiftState.LIFT_INIT;
import static org.firstinspires.ftc.teamcode.RobotConstants.LiftState.LIFT_RETRACT;
import static org.firstinspires.ftc.teamcode.RobotConstants.LiftState.LIFT_START;
import static org.firstinspires.ftc.teamcode.RobotConstants.LiftState.LIFT_STOP;
import static org.firstinspires.ftc.teamcode.autonomous.FieldConstants.ascentParkingBlue;
import static org.firstinspires.ftc.teamcode.autonomous.FieldConstants.blueAllianceBasket;
import static org.firstinspires.ftc.teamcode.autonomous.FieldConstants.blueAllianceBasketStart;
import static org.firstinspires.ftc.teamcode.autonomous.FieldConstants.blueAllianceNeutralCenterSpike;
import static org.firstinspires.ftc.teamcode.autonomous.FieldConstants.blueAllianceNeutralLeftSpike;
import static org.firstinspires.ftc.teamcode.autonomous.FieldConstants.blueAllianceNeutralRightSpike;

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
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathChain;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;
import org.firstinspires.ftc.teamcode.pedroPathing.util.Timer;

@Autonomous(name = "Basket No Specimen Preload + 3", group = "Basket")
public class BasketNoSpec_One_Three extends OpMode {

    private IntakeState intakeState;
    private LiftState liftState;
    private Timer intakeTimer, pathTimer, liftTimer, fullTimer;
    private ElapsedTime opmodeTimer;

    private MecanumTrain bot;

    private int pathState;

    Pose startPose = new Pose(
            blueAllianceBasketStart.getX(),
            blueAllianceBasketStart.getY(),
            blueAllianceBasketStart.getHeading());

    Path preloadBasketScore, rightSampleScore,
            centerSampleScore, leftSampleScore, parkPath;

    PathChain rightSampleCycleChain, centerSampleCycleChain, leftSampleCycleChain;


    @Override
    public void init() {
        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        intakeTimer = new Timer();
        pathTimer = new Timer();
        liftTimer = new Timer();
        opmodeTimer = new ElapsedTime();
        fullTimer = new Timer();

        bot = new MecanumTrain(hardwareMap, opmodeTimer);
        bot.follower.setStartingPose(blueAllianceBasketStart);
        bot.follower.setMaxPower(0.9);


        bot.setIntakeServo("off");
        bot.setIntakePivot("in");
        bot.setHorizontalExtension("in");
    }

    @Override
    public void start() {
        opmodeTimer.reset();
        fullTimer.resetTimer();
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
        telemetry.addData("PathTimer", pathTimer.getElapsedTimeSeconds());
        telemetry.addData("Intake State", intakeState);
        telemetry.addData("IntakeTimer", intakeTimer.getElapsedTimeSeconds());
        telemetry.addData("Lift State", liftState);
        telemetry.addData("LiftTimer", liftTimer.getElapsedTimeSeconds());
        telemetry.addData("fullTimer", fullTimer.getElapsedTimeSeconds());
        telemetry.addData("Vertical Extension", bot.verticalExtension.getCurrentPosition());
        telemetry.addData("Position", bot.follower.getPose());
        telemetry.update();
    }

    private void buildPaths() {
        preloadBasketScore = new Path(new BezierLine(
                new Point(startPose), // 8,84
                new Point(blueAllianceBasket) //17,121,-60
        ));
        preloadBasketScore.setLinearHeadingInterpolation(startPose.getHeading(), blueAllianceBasket.getHeading());

        rightSampleCycleChain = bot.follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Point(blueAllianceBasket), // 17,121
                        new Point(blueAllianceNeutralRightSpike.getX()-36, blueAllianceNeutralRightSpike.getY()-36, Point.CARTESIAN), // 15.5, 91.5
                        new Point(blueAllianceNeutralRightSpike.getX()-15, blueAllianceNeutralRightSpike.getY()-25, Point.CARTESIAN) // 30.5, 96.5
                ))
                .setLinearHeadingInterpolation(Math.toRadians(-60), Math.toRadians(70))
                .addPath(new BezierLine(
                        new Point(blueAllianceNeutralRightSpike.getX()-15, blueAllianceNeutralRightSpike.getY()-25, Point.CARTESIAN), // 30.5, 96.5
                        new Point(blueAllianceNeutralRightSpike.getX()-11.5, blueAllianceNeutralRightSpike.getY()-17, Point.CARTESIAN) // 33.5, 100.5
                ))
                .setLinearHeadingInterpolation(Math.toRadians(70), Math.toRadians(65))
                .build();
        
        rightSampleScore = new Path(new BezierLine(
                new Point(blueAllianceNeutralRightSpike.getX()-10, blueAllianceNeutralRightSpike.getY()-19, Point.CARTESIAN), // 33.5, 100.5
                new Point(blueAllianceBasket.getX()+7, blueAllianceBasket.getY()+4, Point.CARTESIAN))); // 20,124
        rightSampleScore.setLinearHeadingInterpolation(Math.toRadians(70), Math.toRadians(-55));

        centerSampleCycleChain = bot.follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Point(blueAllianceBasket.getX()+7, blueAllianceBasket.getY()+4, Point.CARTESIAN), // 20,124
                        new Point(blueAllianceNeutralCenterSpike.getX()-36, blueAllianceNeutralCenterSpike.getY()-36, Point.CARTESIAN), // 15.5, 95.5
                        new Point(blueAllianceNeutralCenterSpike.getX()-15, blueAllianceNeutralCenterSpike.getY()-25, Point.CARTESIAN)  // 30.5, 97.5
                ))
                .setLinearHeadingInterpolation(Math.toRadians(-60),Math.toRadians(70))
                .addPath(new BezierLine(
                        new Point(blueAllianceNeutralCenterSpike.getX()-15, blueAllianceNeutralCenterSpike.getY()-25, Point.CARTESIAN), // 30.5, 97.5
                        new Point(blueAllianceNeutralCenterSpike.getX()-10, blueAllianceNeutralCenterSpike.getY()-15, Point.CARTESIAN)  // 33.5, 101.5
                ))
                .setLinearHeadingInterpolation(Math.toRadians(70), Math.toRadians(62.5))
                .build();

        centerSampleScore = new Path(new BezierLine(
                new Point(blueAllianceNeutralCenterSpike.getX()-10, blueAllianceNeutralCenterSpike.getY()-15, Point.CARTESIAN),
                new Point(blueAllianceBasket.getX()+8, blueAllianceBasket.getY()+6, Point.CARTESIAN)));
        centerSampleScore.setLinearHeadingInterpolation(Math.toRadians(60), Math.toRadians(-60));

//        leftSampleCycleChain = bot.follower.pathBuilder()
//                .addPath(new BezierCurve(
//                        new Point(blueAllianceBasket.getX()+3, blueAllianceBasket.getY()+3, Point.CARTESIAN), // 20,124
//                        new Point(blueAllianceNeutralLeftSpike.getX()-36, blueAllianceNeutralLeftSpike.getY()-36, Point.CARTESIAN), // 15.5, 105.5
//                        new Point(blueAllianceNeutralLeftSpike.getX()-12, blueAllianceNeutralLeftSpike.getY()-25, Point.CARTESIAN)  // 30.5, 107.5
//                ))
//                .setLinearHeadingInterpolation(Math.toRadians(-60), Math.toRadians(70))
//                .addPath(new BezierLine(
//                        new Point(blueAllianceNeutralLeftSpike.getX()-12, blueAllianceNeutralLeftSpike.getY()-25, Point.CARTESIAN), // 30.5, 107.5
//                        new Point(blueAllianceNeutralLeftSpike.getX()-11, blueAllianceNeutralLeftSpike.getY()-17, Point.CARTESIAN)  // 33.5, 111.5
//                ))
//                .setLinearHeadingInterpolation(Math.toRadians(70), Math.toRadians(70))
//                .build();

        leftSampleCycleChain = bot.follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Point(blueAllianceBasket.getX()+3, blueAllianceBasket.getY()+3, Point.CARTESIAN), // 20,124
                        new Point(blueAllianceNeutralLeftSpike.getX()-36, blueAllianceNeutralLeftSpike.getY()-36, Point.CARTESIAN), // 15.5, 105.5
                        new Point(blueAllianceNeutralLeftSpike.getX()-5, blueAllianceNeutralLeftSpike.getY()-25, Point.CARTESIAN)  // 30.5, 107.5
                ))
                .setLinearHeadingInterpolation(Math.toRadians(-60), Math.toRadians(90))
                .addPath(new BezierLine(
                        new Point(blueAllianceNeutralLeftSpike.getX()-5, blueAllianceNeutralLeftSpike.getY()-25, Point.CARTESIAN), // 30.5, 107.5
                        new Point(blueAllianceNeutralLeftSpike.getX()-3, blueAllianceNeutralLeftSpike.getY()-17, Point.CARTESIAN)  // 33.5, 111.5
                ))
                .setConstantHeadingInterpolation(Math.toRadians(90))
                .build();

        leftSampleScore = new Path(new BezierLine(
                new Point(blueAllianceNeutralLeftSpike.getX()-12, blueAllianceNeutralLeftSpike.getY()-17, Point.CARTESIAN),
                new Point(blueAllianceBasket.getX()+8, blueAllianceBasket.getY()+8, Point.CARTESIAN)));
        leftSampleScore.setLinearHeadingInterpolation(Math.toRadians(70), Math.toRadians(-60));

        parkPath = new Path(new BezierCurve(
                new Point(blueAllianceBasket.getX()+3, blueAllianceBasket.getY()+3, Point.CARTESIAN),
                new Point(ascentParkingBlue.getX(), ascentParkingBlue.getY()+30,Point.CARTESIAN),
                new Point(ascentParkingBlue)
        ));
        parkPath.setLinearHeadingInterpolation(Math.toRadians(-60), ascentParkingBlue.getHeading());
    }

    private void autonomousPathUpdate() {
        switch(pathState) {
            case 1: // Path to the basket (preload); Run the lift and outtake
                bot.follower.followPath(preloadBasketScore);
                if (pathTimer.getElapsedTimeSeconds() > 0.1) {
                    setLiftState(LIFT_START);
                    setPathState(2);
                }
                break;
            case 2:// Path to the right sample cycle; Flip out the intake
                if (liftState == LIFT_RETRACT) {
                    bot.follower.setMaxPower(0.9);
                    bot.follower.followPath(rightSampleCycleChain);
                    setPathState(20);
                }
                break;
            case 20:
                if (pathTimer.getElapsedTimeSeconds() > 0.8) {
                    setIntakeState(INTAKE_FLIP_OUT);
                    setPathState(3);
                }
                break;
            case 3:// Retract the intake
                if (intakeState == INTAKE_RETRACT) {
                    bot.follower.setMaxPower(0.75);
                    bot.follower.followPath(rightSampleScore);
                    setPathState(30);
                }
                break;
            case 30:
                if (pathTimer.getElapsedTimeSeconds() > 0.2) {
                    setLiftState(LIFT_START);
                    setPathState(4);
                }
            case 4: // Path to the center sample cycle
                if (liftState == LIFT_RETRACT) {
                    bot.follower.setMaxPower(0.80);
                    bot.follower.followPath(centerSampleCycleChain);
                    setPathState(40);
                }
                break;
            case 40:
                if (pathTimer.getElapsedTimeSeconds() > 1.0) {
                    setIntakeState(INTAKE_FLIP_OUT);
                    setPathState(5);
                }
                break;
            case 5: // Flip out the intake
                if (intakeState == INTAKE_RETRACT) {
                    bot.follower.setMaxPower(0.65);
                    bot.follower.followPath(centerSampleScore);
                    setPathState(50);
                }
                break;
            case 50:
                if (pathTimer.getElapsedTimeSeconds() > 0.2) {
                    setLiftState(LIFT_START);
                    setPathState(6);
                }
                break;
            case 6: // Retract the intake
                if (liftState == LIFT_RETRACT) {
                    bot.follower.setMaxPower(0.80);
                    bot.follower.followPath(leftSampleCycleChain);
                    setPathState(60);
                }
                break;
            case 60:
                if (pathTimer.getElapsedTimeSeconds() > 0.6) {
                    setIntakeState(INTAKE_FLIP_OUT);
                    setPathState(7);
                }
                break;
            case 7: // Retract the intake
                if (intakeState == INTAKE_RETRACT) {
                    bot.follower.setMaxPower(0.8);
                    bot.follower.followPath(leftSampleScore);
                    setPathState(70);
                }
                break;
            case 70:
                if (pathTimer.getElapsedTimeSeconds() > 0.2) {
                    setLiftState(LIFT_START);
                    setPathState(8);
                }
            case 8: // Path to the park
                if (liftState == LIFT_RETRACT) {
                    bot.follower.setMaxPower(0.8);
                    bot.follower.followPath(parkPath);
                    setPathState(9);
                }

                break;
            case 9: // Stop the opmode
                setLiftState(LIFT_EXTEND_LOW);
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
                bot.setHorizontalExtension("out");
                break;
            case INTAKE_SPIN:
                bot.setIntakeServo("forward");
                if (bot.sampleDetected()) {
                    setIntakeState(INTAKE_SAMPLE_IN);
                }
                break;
            case INTAKE_SAMPLE_IN:
                if (intakeTimer.getElapsedTimeSeconds() > 0.1) {
                    bot.setIntakeServo("off");
                    setIntakeState(INTAKE_RETRACT);
                }
                break;
            case INTAKE_RETRACT:
                bot.setHorizontalExtension("in");
                if (intakeTimer.getElapsedTimeSeconds() > 0.1) {
                    setIntakeState(INTAKE_FLIP_IN);
                }
                break;
            case INTAKE_FLIP_IN:
                bot.setIntakePivot("in");
                if (intakeTimer.getElapsedTimeSeconds() > 0.8) {
                    setIntakeState(INTAKE_SAMPLE_OUT);
                }
                break;
            case INTAKE_SAMPLE_OUT:
                if(intakeTimer.getElapsedTimeSeconds() > 0.3) {
                    bot.setIntakeServo("backward");
                    if (bot.sampleInOuttake()) {
                        setIntakeState(INTAKE_STOP);
                    }
                }
                break;
            case INTAKE_STOP:
                if(intakeTimer.getElapsedTimeSeconds() > 0.3) {
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
                if (bot.verticalExtension.getCurrentPosition() >= 1700) {
                    setLiftState(BUCKET_TIP);
                }
                break;
            case LIFT_EXTEND_HIGH:
                    bot.liftExtend_highBucket();
                    if (bot.verticalExtension.getCurrentPosition() >= 3960) {
                        setLiftState(BUCKET_TIP);
                    }
                break;
            case BUCKET_TIP:
                bot.setBucket("tip");
                if (!bot.sampleInOuttake() && liftTimer.getElapsedTimeSeconds() >= 1.1) {
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
        }}}
   