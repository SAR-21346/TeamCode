package org.firstinspires.ftc.teamcode.autonomous;

import static org.firstinspires.ftc.teamcode.RobotConstants.INTAKE_POWER_NEG;
import static org.firstinspires.ftc.teamcode.RobotConstants.INTAKE_POWER_POS;
import static org.firstinspires.ftc.teamcode.RobotConstants.IntakeState.EXTEND;
import static org.firstinspires.ftc.teamcode.RobotConstants.IntakeState.INTAKE_ACCEPT;
import static org.firstinspires.ftc.teamcode.RobotConstants.IntakeState.INTAKE_ENABLE;
import static org.firstinspires.ftc.teamcode.RobotConstants.IntakeState.INTAKE_REJECT;
import static org.firstinspires.ftc.teamcode.RobotConstants.IntakeState.PIVOT_DOWN;
import static org.firstinspires.ftc.teamcode.RobotConstants.IntakeState.PIVOT_DOWN_BYPASS;
import static org.firstinspires.ftc.teamcode.RobotConstants.IntakeState.PIVOT_UP;
import static org.firstinspires.ftc.teamcode.RobotConstants.IntakeState.STOP;
import static org.firstinspires.ftc.teamcode.RobotConstants.LIFT_HIGH_BUCKET;
import static org.firstinspires.ftc.teamcode.RobotConstants.OuttakeState.EXTEND_HIGH_BUCKET;
import static org.firstinspires.ftc.teamcode.RobotConstants.OuttakeState.EXTEND_HIGH_SPEC;
import static org.firstinspires.ftc.teamcode.RobotConstants.OuttakeState.INIT;
import static org.firstinspires.ftc.teamcode.RobotConstants.OuttakeState.INTAKE_GRAB;
import static org.firstinspires.ftc.teamcode.RobotConstants.OuttakeState.RETRACT;
import static org.firstinspires.ftc.teamcode.RobotConstants.OuttakeState.SCORE_HIGH_BUCKET;
import static org.firstinspires.ftc.teamcode.RobotConstants.OuttakeState.SCORE_HIGH_SPEC;
import static org.firstinspires.ftc.teamcode.RobotConstants.OuttakeState.SPEC_PICKUP;
import static org.firstinspires.ftc.teamcode.RobotConstants.OuttakeState.START;
import static org.firstinspires.ftc.teamcode.RobotConstants.ROBOT_CENTER_TO_MAX_EXT;
import static org.firstinspires.ftc.teamcode.autonomous.FieldConstants.ascentParkingBlue;
import static org.firstinspires.ftc.teamcode.autonomous.FieldConstants.basket;
import static org.firstinspires.ftc.teamcode.autonomous.FieldConstants.basketStart;
import static org.firstinspires.ftc.teamcode.autonomous.FieldConstants.neutralCenterSpike;
import static org.firstinspires.ftc.teamcode.autonomous.FieldConstants.neutralLeftSpike;
import static org.firstinspires.ftc.teamcode.autonomous.FieldConstants.neutralRightSpike;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.MecanumTrain;
import org.firstinspires.ftc.teamcode.RobotConstants;
import org.firstinspires.ftc.teamcode.RobotConstants.IntakeState;
import org.firstinspires.ftc.teamcode.RobotConstants.OuttakeState;
@Autonomous(name = "0+4 BASKET", group = "Basket")
public class Basket_Zero_Four extends OpMode {

    private IntakeState intakeState;
    private OuttakeState outtakeState;
    private Timer intakeTimer, pathTimer, outtakeTimer, fullTimer;
    private MecanumTrain bot;
    private int pathState;

    Pose startPose = new Pose(
            basketStart.getX(),
            basketStart.getY(),
            basketStart.getHeading());

    PathChain preload, cycle1, cycle1Score, cycle2, cycle2Score, cycle3, cycle3Score, park;

    @Override
    public void init() {
        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        intakeTimer = new Timer();
        pathTimer = new Timer();
        outtakeTimer = new Timer();
        fullTimer = new Timer();

        bot = new MecanumTrain(hardwareMap);
        bot.follower.setStartingPose(basketStart);
        setIntakeState(IntakeState.INIT);
        bot.outtake_flat();
        setOuttakeState(INIT);
    }

    @Override
    public void start() {
        fullTimer.resetTimer();
        buildPaths();
        setPathState(1);


    }

    @Override
    public void loop() {
        bot.follower.update();

        bot.updateLift();
        bot.distSensorUpdate();
        autonomousPathUpdate();
        intakeStateUpdate();
        outtakeStateUpdate();

        telemetry.addData("TValue", bot.follower.getCurrentTValue());
        telemetry.addData("Path State", pathState);
        telemetry.addData("PathTimer", pathTimer.getElapsedTimeSeconds());
        telemetry.addData("Intake State", intakeState);
        telemetry.addData("IntakeTimer", intakeTimer.getElapsedTimeSeconds());
        telemetry.addData("outtake state",outtakeState);
        telemetry.addData("LiftTimer", outtakeTimer.getElapsedTimeSeconds());
        telemetry.addData("fullTimer", fullTimer.getElapsedTimeSeconds());
        telemetry.addData("Position", bot.follower.getPose());
        telemetry.update();
    }

    private void buildPaths() {
        preload = bot.follower.pathBuilder()
                .addPath(new Path(new BezierCurve(
                            new Point(basketStart),
                            new Point(28, 105.75, Point.CARTESIAN),
                            new Point(23.5, 116.5, Point.CARTESIAN))))
                .setLinearHeadingInterpolation(basketStart.getHeading(), basket.getHeading())
                .addParametricCallback(0.05, () -> setOuttakeState(INTAKE_GRAB))
                .addPath(new Path(new BezierLine(
                        new Point(23.5, 116.5, Point.CARTESIAN),
                        new Point(basket))))
                .setConstantHeadingInterpolation(basket.getHeading())
                .addParametricCallback(0.65, () -> setIntakeState(EXTEND))
                .setPathEndHeadingConstraint(3.0)
                .build();

        cycle1 = bot.follower.pathBuilder()
                .addPath(new Path(new BezierLine(
                        new Point(basket),
                        new Point(neutralRightSpike.getX() - 9, neutralRightSpike.getY(), Point.CARTESIAN))))
                .setLinearHeadingInterpolation(basket.getHeading(), neutralRightSpike.getHeading(), 0.6)
                .addPath(new Path(new BezierLine(new Point(basket), new Point(neutralRightSpike))))
                .setConstantHeadingInterpolation(neutralRightSpike.getHeading())
                .setPathEndVelocityConstraint(20)
                .setPathEndTimeoutConstraint(50)
                .build();

        cycle1Score = bot.follower.pathBuilder()
                .addPath(new Path(new BezierLine(new Point(neutralRightSpike),new Point(basket))))
                .setLinearHeadingInterpolation(neutralRightSpike.getHeading(), basket.getHeading())
                .addParametricCallback(0.5, () -> setOuttakeState(START))
                .build();

        cycle2 = bot.follower.pathBuilder()
                .addPath(new Path(new BezierLine(
                        new Point(basket),
                        new Point(neutralCenterSpike.getX() - 4, neutralCenterSpike.getY(), Point.CARTESIAN))))
                .setLinearHeadingInterpolation(basket.getHeading(), neutralCenterSpike.getHeading(), 0.05)
                .addParametricCallback(0.1, () -> setIntakeState(EXTEND))
                .addPath(new Path(new BezierLine(new Point(basket), new Point(neutralCenterSpike))))
                .setConstantHeadingInterpolation(neutralCenterSpike.getHeading())
                .setPathEndTimeoutConstraint(50)
                .setZeroPowerAccelerationMultiplier(2)
                .build();

        cycle2Score = bot.follower.pathBuilder()
                .addPath(new Path(new BezierLine(new Point(neutralCenterSpike), new Point(basket))))
                .setLinearHeadingInterpolation(neutralCenterSpike.getHeading(), basket.getHeading())
                .addParametricCallback(0.5, () -> setOuttakeState(START))
                .build();

        cycle3 = bot.follower.pathBuilder()
                .addPath(new Path(new BezierLine(
                                    new Point(basket), // start
                                    new Point(neutralLeftSpike.getX(), neutralLeftSpike.getY()-15, Point.CARTESIAN) // end
                        )
                ))
                .setLinearHeadingInterpolation(basket.getHeading(), neutralLeftSpike.getHeading())
                .addParametricCallback(0.5, () -> setIntakeState(EXTEND))
                .addPath(new Path(new BezierLine(
                        new Point(neutralLeftSpike.getX(), neutralLeftSpike.getY()-15, Point.CARTESIAN),
                        new Point(neutralLeftSpike)
                )))
                .setConstantHeadingInterpolation(neutralLeftSpike.getHeading())
                .addPath(new Path(new BezierLine(
                        new Point(neutralLeftSpike),
                        new Point(neutralLeftSpike.getX(), neutralLeftSpike.getY()-1, Point.CARTESIAN)
                )))
                .setLinearHeadingInterpolation(neutralLeftSpike.getHeading(), neutralLeftSpike.getHeading() + Math.toRadians(5))
                .setPathEndTimeoutConstraint(140)
                .build();


        cycle3Score = bot.follower.pathBuilder()
                .addPath(new Path(
                        new BezierLine(
                                new Point(neutralLeftSpike.getX(), neutralLeftSpike.getY()-1, Point.CARTESIAN), // start
                                new Point(basket)))
                )
                .setLinearHeadingInterpolation(neutralLeftSpike.getHeading(), basket.getHeading())
                .addParametricCallback(0.75, () -> setOuttakeState(START))
                .build();

        park = bot.follower.pathBuilder()
                .addPath(new Path(
                        new BezierCurve(
                                new Point(basket), // start
                                new Point(60, 130, Point.CARTESIAN), // control point 1
                                new Point(ascentParkingBlue)))
                )
                .setLinearHeadingInterpolation(basket.getHeading(), ascentParkingBlue.getHeading())
                .build();
    }

    private void autonomousPathUpdate() {
        switch (pathState) {
            case 1: // preload
                bot.follower.followPath(preload, true);
                setPathState(2);
                break;
            case 2: // outtake fsm end, begin cycle 1 path
                if (!bot.follower.isBusy() && outtakeState == RETRACT) {
                    bot.follower.followPath(cycle1, true);
                    setPathState(3);
                }
                break;
            case 3: // drive to bucket
                if (!bot.follower.isBusy() && intakeState == PIVOT_UP) {
                    bot.follower.followPath(cycle1Score, 0.85, true);
                    setPathState(4);
                }
                break;
            case 4: // outtake fsm end, begin cycle 1 path
                if (!bot.follower.isBusy() && outtakeState == RETRACT) {
                    bot.follower.followPath(cycle2, 0.9, true);
                    setPathState(5);
                }
                break;
            case 5: // drive to bucket
                if (!bot.follower.isBusy() && intakeState == PIVOT_UP) {
                    bot.follower.followPath(cycle2Score, 0.75, true);
                    setPathState(6);
                }
                break;
            case 6: // outtake fsm end
                if (outtakeState == RETRACT) {
                    bot.follower.followPath(cycle3, 0.75, true);
                    setPathState(7);
                }
                break;
            case 7: // drive to bucket
                if (!bot.follower.isBusy() && intakeState == PIVOT_UP) {
                    bot.follower.followPath(cycle3Score, true);
                    setPathState(7);
                }
                break;
            case 8: // outtake fsm end, begin cycle 1 path
                if (!bot.follower.isBusy() && outtakeState == RETRACT) {
                    bot.follower.followPath(park, 0.5, true);
                    setPathState(5);
                }
                break;
        }
    }

    private void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
        autonomousPathUpdate();
    }

    private void setIntakeState(IntakeState iState) {
        intakeState = iState;
        intakeTimer.resetTimer();
        intakeStateUpdate();
    }

    private void setOuttakeState(RobotConstants.OuttakeState oState) {
        outtakeState = oState;
        outtakeTimer.resetTimer();
        outtakeStateUpdate();
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
                bot.intake_extend(3);
                setIntakeState(PIVOT_DOWN);
                break;
            case PIVOT_DOWN:
               bot.intake_pivot_down();
               bot.outtake_clearance();
               setIntakeState(INTAKE_ENABLE);
               break;
            case PIVOT_DOWN_BYPASS:
                bot.intake_pivot_down();
                bot.outtake_clearance();
                setIntakeState(INTAKE_ENABLE);
                break;
            case INTAKE_ENABLE:
                // Pick up sample
                bot.intake.setPower(.6);
                bot.distSensorUpdate();
                if (bot.intakeWallDetect()) {
                    setIntakeState(INTAKE_ACCEPT);
                }
                break;
            case INTAKE_ACCEPT:
                bot.distSensorUpdate();
                if (intakeTimer.getElapsedTimeSeconds() > 0.3) {
                    if (bot.intakeWallDetect()) {
                        bot.intake.setPower(0);
                        setIntakeState(PIVOT_UP);
                    }
                }
            case PIVOT_UP:
                // pivot intake up
                bot.intake_pivot_up();
                // retract slides
                bot.intake_retract();
                bot.intake.setPower(0);
                if (intakeTimer.getElapsedTimeSeconds() > 0.8) {
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
                if (outtakeTimer.getElapsedTimeSeconds() > 0.8) { setOuttakeState(INTAKE_GRAB); }
                break;
            case INTAKE_GRAB:
                bot.outtake_flat();
                if(bot.intakeWallDetect()) { // Checks if something is in the intake
                    bot.claw_close();
                    setOuttakeState(EXTEND_HIGH_BUCKET);
                }
                break;
            case EXTEND_HIGH_BUCKET:
                // Extend it to top basket
                bot.extend_high_bucket();
                bot.outtake_score_bucket();
                if (bot.liftR.getCurrentPosition() >= LIFT_HIGH_BUCKET-50) {
                    setOuttakeState(SCORE_HIGH_BUCKET);
                }
                break;
            case SCORE_HIGH_BUCKET:
                bot.claw_open();
                setOuttakeState(RETRACT);
                break;
            case RETRACT:
                // wait for claw
                if (outtakeTimer.getElapsedTimeSeconds() > 1.4) {
                    bot.outtake_flat();
                    setOuttakeState(OuttakeState.STOP);
                }
            case STOP:
                bot.outtake_clearance();
                if (bot.liftR.getCurrentPosition() > 200) {
                    bot.retractLift();
                } else {
                    bot.resetLift();
                }
                bot.claw_open();
                break;

        }
    }
}
