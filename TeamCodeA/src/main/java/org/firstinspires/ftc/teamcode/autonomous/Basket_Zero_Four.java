package org.firstinspires.ftc.teamcode.autonomous;

import static org.firstinspires.ftc.teamcode.RobotConstants.INTAKE_POWER_NEG;
import static org.firstinspires.ftc.teamcode.RobotConstants.INTAKE_POWER_POS;
import static org.firstinspires.ftc.teamcode.RobotConstants.IntakeState.EXTEND;
import static org.firstinspires.ftc.teamcode.RobotConstants.IntakeState.INTAKE_ACCEPT;
import static org.firstinspires.ftc.teamcode.RobotConstants.IntakeState.INTAKE_ENABLE;
import static org.firstinspires.ftc.teamcode.RobotConstants.IntakeState.INTAKE_REJECT;
import static org.firstinspires.ftc.teamcode.RobotConstants.IntakeState.PIVOT_DOWN;
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
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

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
    }

    @Override
    public void start() {
        fullTimer.resetTimer();
        buildPaths();
        setPathState(1);
        setIntakeState(IntakeState.INIT);
        setOuttakeState(INIT);
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
        telemetry.addData("LiftTimer", outtakeTimer.getElapsedTimeSeconds());
        telemetry.addData("fullTimer", fullTimer.getElapsedTimeSeconds());
        telemetry.addData("Position", bot.follower.getPose());
        telemetry.update();
    }

    private void buildPaths() {
        preload = bot.follower.pathBuilder()
                .addPath(new Path(
                        new BezierCurve(
                                new Point(basketStart),
                                new Point(28.25, 117, Point.CARTESIAN),
                                new Point(basket))))
                .setLinearHeadingInterpolation(basketStart.getHeading(), basket.getHeading())
                .setPathEndHeadingConstraint(3.0)
                .build();

        cycle1 = bot.follower.pathBuilder()
                .addPath(new Path(
                                new BezierCurve(new Point(basket), // start
                                                new Point(13, 124, Point.CARTESIAN), // control point 1
                                                new Point(neutralRightSpike.getX(), neutralRightSpike.getY(), Point.CARTESIAN))
                ))
                .setLinearHeadingInterpolation(basket.getHeading(), neutralRightSpike.getHeading())
                .build();

        cycle1Score = bot.follower.pathBuilder()
                .addPath(new Path(new BezierLine(new Point(neutralRightSpike),new Point(basket))))
                .setLinearHeadingInterpolation(neutralRightSpike.getHeading(), basket.getHeading())
                .build();

        cycle2 = bot.follower.pathBuilder()
                .addPath(new Path(new BezierLine(new Point(basket), new Point(neutralCenterSpike))))
                .setLinearHeadingInterpolation(basket.getHeading(), neutralCenterSpike.getHeading())
                .build();

        cycle2Score = bot.follower.pathBuilder()
                .addPath(new Path(
                                new BezierLine(
                                        new Point(neutralCenterSpike.getX()-ROBOT_CENTER_TO_MAX_EXT, neutralCenterSpike.getY(), Point.CARTESIAN), // start
                                        new Point(basket))
                ))
                .setLinearHeadingInterpolation(neutralCenterSpike.getHeading(), basket.getHeading())
                .build();

        cycle3 = bot.follower.pathBuilder()
                .addPath(new Path(
                                new BezierCurve(
                                                new Point(basket), // start
                                                new Point(10, 113, Point.CARTESIAN), // control point 1
                                                new Point(34, 95.75, Point.CARTESIAN), // control point 2
                                                new Point(neutralLeftSpike.getX(), neutralLeftSpike.getY()-ROBOT_CENTER_TO_MAX_EXT, Point.CARTESIAN) // end
                        )
                ))
                .setLinearHeadingInterpolation(basket.getHeading(), neutralLeftSpike.getHeading())
                .build();


        cycle3Score = bot.follower.pathBuilder()
                .addPath(new Path(
                        new BezierLine(
                                new Point(neutralLeftSpike.getX(), neutralLeftSpike.getY()-ROBOT_CENTER_TO_MAX_EXT, Point.CARTESIAN), // start
                                new Point(basket)))
                )
                .setLinearHeadingInterpolation(neutralCenterSpike.getHeading(), basket.getHeading())
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
                setPathState(10);
                break;
            case 10: // outtake fsm begin, score preload
                if (bot.follower.getCurrentTValue() > 0.10) {
                    setOuttakeState(INTAKE_GRAB);
                    setPathState(2);
                }
                break;
            case 2: // outtake fsm end, begin cycle 1 path
                if (outtakeState == RETRACT) {
                    bot.follower.followPath(cycle1, true);
                    setPathState(20);
                }
                break;
            case 20: // intake fsm begin
                if (pathTimer.getElapsedTimeSeconds() > 0.2) {
                    setIntakeState(EXTEND);
                    setPathState(21);
                }
                break;
            case 21: // intake fsm end, outtake fsm begin
                if (intakeState == PIVOT_UP) {
                    setOuttakeState(INTAKE_GRAB);
                    setPathState(3);
                }
            case 3: // drive to bucket
                if (outtakeState == EXTEND_HIGH_BUCKET) {
                    bot.follower.followPath(cycle1Score, true);
                    setPathState(4);
                }
                break;
            case 4: // outtake fsm end
                if (outtakeState == RETRACT) {
                    bot.follower.followPath(cycle2, true);
                    setPathState(20);
                }
                break;
            case 40: // intake fsm begin, begin cycle 2
                if (pathTimer.getElapsedTimeSeconds() > 0.2) {
                    setIntakeState(EXTEND);
                    setPathState(41);
                }
                break;
            case 41: // intake fsm end, outtake fsm begin
                if (intakeState == PIVOT_UP) {
                    setOuttakeState(INTAKE_GRAB);
                    setPathState(5);
                }
            case 5: // begin cycle 2 score
                if (outtakeState == EXTEND_HIGH_BUCKET) {
                    bot.follower.followPath(cycle2Score, true);
                    setPathState(6);
                }
                break;
            case 6: // outtake fsm end, begin cycle 3
                if (outtakeState == RETRACT) {
                    bot.follower.followPath(cycle3, true);
                    setPathState(60);
                }
                break;
            case 60:
                if (pathTimer.getElapsedTimeSeconds() > 0.2) {
                    setIntakeState(EXTEND);
                    setPathState(61);
                }
                break;
            case 61:
                if (intakeState == PIVOT_UP) {
                    setOuttakeState(INTAKE_GRAB);
                    setPathState(7);
                }
            case 7:
                if (outtakeState == EXTEND_HIGH_BUCKET) {
                    bot.follower.followPath(cycle3Score, true);
                     setPathState(8);
                }
                break;
            case 8:
                if (outtakeState == RETRACT) {
                    bot.follower.followPath(park, true);
                    setPathState(-1);
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
                bot.retract();
                bot.pivot_up();
                break;
            case EXTEND:
                // Extend horizontal slides
                bot.extend(5);
                setIntakeState(PIVOT_DOWN);
            case PIVOT_DOWN:
                if(intakeTimer.getElapsedTimeSeconds() > 0.4) { // replace "true" with slide limit check
                    bot.pivot_down();
                    bot.outtake_clearance();
                    setIntakeState(INTAKE_ENABLE);
                }
                break;
            case INTAKE_ENABLE:
                // Pick up sample
                bot.intake.setPower(INTAKE_POWER_POS);
                bot.distSensorUpdate();
                if (bot.intakeWheelDist < 25 || bot.intakeWallDetect()) {
                   setIntakeState(INTAKE_ACCEPT);
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
                bot.pivot_up();
                // retract slides
                bot.retract();
                bot.intake.setPower(0);
                if (intakeTimer.getElapsedTimeSeconds() > 0.8) {
                    setIntakeState(STOP);
                }
                break;
            case STOP:
                // pivot intake up
                bot.pivot_up();
                // retract slides
                bot.retract();
                bot.outtake_flat();
                bot.intake.setPower(0);
                break;
        }
    }

    private void outtakeStateUpdate() {
        switch (outtakeState) {
            case INIT:
                // make sure outtake is reset to right pos
                bot.resetLift();
                bot.outtake_flat();
                bot.claw_open();
                break;
            case INTAKE_GRAB:
                if(bot.intakeWallDetect()) { // Checks if something is in the intake
                    bot.claw_close();
                    setOuttakeState(EXTEND_HIGH_BUCKET);
                }
                break;
            case EXTEND_HIGH_BUCKET:
                // Extend it to top basket
                bot.extend_high_bucket();
                if (bot.liftR.getCurrentPosition() >= LIFT_HIGH_BUCKET-30) { // replace 10 with height of vert ext
                    setOuttakeState(SCORE_HIGH_BUCKET);
                }
                break;
            case SCORE_HIGH_BUCKET:
                bot.outtake_score_bucket();
                if (outtakeTimer.getElapsedTimeSeconds() > 1) { // replace with encoder value of outtake servo
                    bot.claw_open();
                    setOuttakeState(RETRACT);
                }
                break;
            case RETRACT:
                // wait for claw
                bot.outtake_flat();
                bot.retractLift();

                if (bot.liftR.getCurrentPosition() < 300) {
                    setOuttakeState(OuttakeState.STOP);
                }
                 break;
            case STOP:
                bot.outtake_flat();
                bot.resetLift();
                bot.claw_open();
                break;
        }
    }
}
