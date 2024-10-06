package org.firstinspires.ftc.teamcode.autonomous;

import static org.firstinspires.ftc.teamcode.RobotConstants.IntakeState.INTAKE_FLIP_IN;
import static org.firstinspires.ftc.teamcode.RobotConstants.IntakeState.INTAKE_INIT;
import static org.firstinspires.ftc.teamcode.RobotConstants.IntakeState.INTAKE_RELEASE;
import static org.firstinspires.ftc.teamcode.RobotConstants.IntakeState.INTAKE_RETRACT;
import static org.firstinspires.ftc.teamcode.RobotConstants.IntakeState.INTAKE_SAMPLE_IN;
import static org.firstinspires.ftc.teamcode.RobotConstants.IntakeState.INTAKE_SPIN;
import static org.firstinspires.ftc.teamcode.RobotConstants.IntakeState.INTAKE_START;
import static org.firstinspires.ftc.teamcode.autonomous.FieldConstants.ascentParking;
import static org.firstinspires.ftc.teamcode.autonomous.FieldConstants.blueAllianceBasket;
import static org.firstinspires.ftc.teamcode.autonomous.FieldConstants.blueAllianceBasketStart;
import static org.firstinspires.ftc.teamcode.autonomous.FieldConstants.blueAllianceBlueCenterSpike;
import static org.firstinspires.ftc.teamcode.autonomous.FieldConstants.blueAllianceBlueLeftSpike;
import static org.firstinspires.ftc.teamcode.autonomous.FieldConstants.blueAllianceBlueRightSpike;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.MecanumTrain;
import org.firstinspires.ftc.teamcode.RobotConstants.IntakeState;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierCurve;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;
import org.firstinspires.ftc.teamcode.pedroPathing.util.Timer;

@Autonomous(name = "Blue Basket No Specimen Preload + 3", group = "Blue Basket")
public class BlueBasketNoSpec extends OpMode {

    private IntakeState intakeState;
    private Timer intakeTimer, pathTimer;
    private ElapsedTime opmodeTimer;

    private MecanumTrain bot;

    private int pathState;

    private Pose startPose = new Pose(
            blueAllianceBasketStart.getX(),
            blueAllianceBasketStart.getY(),
            blueAllianceBasketStart.getHeading());

    Path preloadBasketScore, rightSampleCycle, rightSampleScore,
            centerSampleCycle, centerSampleScore, leftSampleCycle, leftSampleScore, parkPath;


    @Override
    public void init() {
        intakeTimer = new Timer();
        pathTimer = new Timer();
        opmodeTimer = new ElapsedTime();

        bot = new MecanumTrain(hardwareMap, opmodeTimer);

        bot.setIntakeServo("off");
        bot.setIntakePivot("in");
        bot.setHorizontalExtension("in");
    }

    @Override
    public void start() {
        setIntakeState(INTAKE_INIT);
        opmodeTimer.reset();
        buildPaths();
        setPathState(1);
    }
    @Override
    public void loop() {
        bot.follower.update();

        autonomousPathUpdate();
        intakeStateUpdate();
    }

    private void buildPaths() {
        preloadBasketScore = new Path(new BezierLine(
                new Point(startPose),
                new Point(blueAllianceBasket)
        ));
        preloadBasketScore.setLinearHeadingInterpolation(startPose.getHeading(), blueAllianceBasket.getHeading());
        preloadBasketScore.setPathEndTimeoutConstraint(0);

        rightSampleCycle = new Path(new BezierCurve(
                new Point(blueAllianceBasket),
                new Point(22.0, 114.0, Point.CARTESIAN),
                new Point(48.5, 96.0, Point.CARTESIAN),
                new Point(blueAllianceBlueRightSpike.getX(), blueAllianceBlueRightSpike.getY()-8.0, Point.CARTESIAN)
        ));
        rightSampleCycle.setLinearHeadingInterpolation(blueAllianceBasket.getHeading(), Math.toRadians(90));
        rightSampleCycle.setPathEndTimeoutConstraint(0);

        rightSampleScore = new Path(new BezierLine(
                new Point(blueAllianceBlueRightSpike.getX(), blueAllianceBlueRightSpike.getY()-8.0, Point.CARTESIAN),
                new Point(blueAllianceBasket)));
        rightSampleScore.setLinearHeadingInterpolation(Math.toRadians(90), blueAllianceBasket.getHeading());
        rightSampleScore.setPathEndTimeoutConstraint(0);

        centerSampleCycle = new Path(new BezierCurve(
                new Point(blueAllianceBasket),
                new Point(35.0, 120.0, Point.CARTESIAN),
                new Point(46.8, 107.0, Point.CARTESIAN),
                new Point(blueAllianceBlueCenterSpike.getX(), blueAllianceBlueCenterSpike.getY()-8.0, Point.CARTESIAN)
        ));
        centerSampleCycle.setLinearHeadingInterpolation(blueAllianceBasket.getHeading(), Math.toRadians(90));
        centerSampleCycle.setPathEndTimeoutConstraint(0);

        centerSampleScore = new Path(new BezierLine(
                new Point(blueAllianceBlueCenterSpike.getX(), blueAllianceBlueCenterSpike.getY()-8.0, Point.CARTESIAN),
                new Point(blueAllianceBasket)));
        centerSampleScore.setLinearHeadingInterpolation(Math.toRadians(90), blueAllianceBasket.getHeading());
        centerSampleScore.setPathEndTimeoutConstraint(0);

        leftSampleCycle = new Path(new BezierCurve(
                new Point(blueAllianceBasket),
                new Point(44.0, 117.0, Point.CARTESIAN),
                new Point(blueAllianceBlueLeftSpike.getX(), blueAllianceBlueLeftSpike.getY()-8.0, Point.CARTESIAN)
        ));
        leftSampleCycle.setLinearHeadingInterpolation(blueAllianceBasket.getHeading(), Math.toRadians(90));
        leftSampleCycle.setPathEndTimeoutConstraint(0);

        leftSampleScore = new Path(new BezierLine(
                new Point(blueAllianceBlueLeftSpike.getX(), blueAllianceBlueLeftSpike.getY()-8.0, Point.CARTESIAN),
                new Point(blueAllianceBasket)));
        leftSampleScore.setLinearHeadingInterpolation(Math.toRadians(90), blueAllianceBasket.getHeading());
        leftSampleScore.setPathEndTimeoutConstraint(0);

        parkPath = new Path(new BezierCurve(
                new Point(blueAllianceBasket),
                new Point(ascentParking.getX(), 120, Point.CARTESIAN),
                new Point(ascentParking)
        ));
        parkPath.setLinearHeadingInterpolation(blueAllianceBasket.getHeading(), ascentParking.getHeading());
        parkPath.setPathEndTimeoutConstraint(0);
    }

    private void autonomousPathUpdate() {
        switch(pathState) {
            case 1:
                break;
            case 2:
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
