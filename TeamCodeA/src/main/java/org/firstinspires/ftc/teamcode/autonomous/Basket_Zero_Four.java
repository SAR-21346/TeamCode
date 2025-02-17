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

    Path startToBasket, basketToCycle1, cycle1toBasket, basketToCycle2, cycle2ToBasket,
            basketToCycle3, cycle3ToBasket, basketToPark;
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
        startToBasket = new Path(new BezierLine(new Point(basketStart), new Point(basket)));
        preload = bot.follower.pathBuilder()
                .addPath(startToBasket)
                .setLinearHeadingInterpolation(basket.getHeading(), Math.toRadians(315))
                .build();

        basketToCycle1 = new Path(
                    new BezierCurve(new Point(basket), // start
                    new Point(15, 121, Point.CARTESIAN), // control point 1
                    new Point(neutralRightSpike.getX()-ROBOT_CENTER_TO_MAX_EXT, neutralRightSpike.getY(), Point.CARTESIAN))
        ); // end
        cycle1 = bot.follower.pathBuilder()
                .addPath(basketToCycle1)
                .setLinearHeadingInterpolation(basket.getHeading(), Math.toRadians(0))
                .build();

        cycle1toBasket = new Path(
                new BezierLine(new Point(neutralRightSpike.getX()-ROBOT_CENTER_TO_MAX_EXT, neutralRightSpike.getY(), Point.CARTESIAN), // start
                new Point(basket))); // end
        cycle1Score = bot.follower.pathBuilder()
                .addPath(cycle1toBasket)
                .setLinearHeadingInterpolation(Math.toRadians(0), basket.getHeading())
                .build();

        basketToCycle2 = new Path(
                new BezierCurve(
                        new Point(basket), // start
                        new Point(16, 132, Point.CARTESIAN), // control point 1
                        new Point(neutralCenterSpike.getX()-ROBOT_CENTER_TO_MAX_EXT, neutralCenterSpike.getY(), Point.CARTESIAN)
                )
        ); // end
        cycle2 = bot.follower.pathBuilder()
                .addPath(basketToCycle2)
                .setLinearHeadingInterpolation(basket.getHeading(), Math.toRadians(0))
                .build();

        cycle2ToBasket = new Path(
                new BezierLine(new Point(neutralCenterSpike.getX()-ROBOT_CENTER_TO_MAX_EXT, neutralCenterSpike.getY(), Point.CARTESIAN), // start
                new Point(basket))); // end
        cycle2Score = bot.follower.pathBuilder()
                .addPath(cycle2ToBasket)
                .setLinearHeadingInterpolation(Math.toRadians(0), basket.getHeading())
                .build();

        basketToCycle3 = new Path(
                new BezierCurve(
                        new Point(basket), // start
                        new Point(17.75, 110, Point.CARTESIAN), // control point 1
                        new Point(47, 100, Point.CARTESIAN), // control point 2
                        new Point(neutralLeftSpike.getX(), neutralLeftSpike.getY()-ROBOT_CENTER_TO_MAX_EXT, Point.CARTESIAN) // end
                )
        );
        cycle3 = bot.follower.pathBuilder()
                .addPath(basketToCycle3)
                .setLinearHeadingInterpolation(basket.getHeading(), Math.toRadians(90))
                .build();

        cycle3ToBasket = new Path(
                new BezierLine(new Point(neutralLeftSpike.getX(), neutralLeftSpike.getY()-ROBOT_CENTER_TO_MAX_EXT, Point.CARTESIAN), // start
                new Point(basket))); // end
        cycle3Score = bot.follower.pathBuilder()
                .addPath(cycle3ToBasket)
                .setLinearHeadingInterpolation(Math.toRadians(90), basket.getHeading())
                .build();

        basketToPark = new Path(
                new BezierCurve(
                        new Point(basket), // start
                        new Point(60, 130, Point.CARTESIAN), // control point 1
                        new Point(ascentParkingBlue))); // end
        park = bot.follower.pathBuilder()
                .addPath(basketToPark)
                .setLinearHeadingInterpolation(basket.getHeading(), ascentParkingBlue.getHeading())
                .build();
    }

    private void autonomousPathUpdate() {
        switch (pathState) {

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
            case DISTANCE_CHECK:
                // Check distance
                if (true) { // replace "true" with distance check
                    setIntakeState(EXTEND);
                }
                break;
            case EXTEND:
                // Extend horizontal slides
                bot.extend(5);
                setIntakeState(PIVOT_DOWN);
            case PIVOT_DOWN:
                if(bot.extRPos < 200) { // replace "true" with slide limit check
                    bot.pivot_down();
                    setIntakeState(INTAKE_ENABLE);
                }
                break;
            case PIVOT_DOWN_BYPASS:
                bot.pivot_down();
                setIntakeState(INTAKE_ENABLE);
            case INTAKE_ENABLE:
                // Pick up sample
                bot.intake.setPower(INTAKE_POWER_POS);
                bot.distSensorUpdate();
                if (bot.intakeWheelDist < 25 || bot.intakeWallDetect()) {
                    if (bot.intakeWheelDetect() == 1 || bot.intakeWheelDetect() == 3) { // 2 for blue, 3 for yellow
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
                if (bot.extRPos > 200) {
                    setIntakeState(STOP);
                }
                break;
            case STOP:
                // pivot intake up
                bot.pivot_up();
                // retract slides
                bot.retract();
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
            case START:
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
                if (bot.liftR.getCurrentPosition() >= LIFT_HIGH_BUCKET-30) { // replace 10 with height of vert ext
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
                if (gamepad2.dpad_left) {
                    bot.claw_close();
                    setOuttakeState(EXTEND_HIGH_SPEC);
                }
                break;
            case EXTEND_HIGH_SPEC:
                // Score it to top rung
                bot.outtake_score_spec();
                if (true) { // replace with encoder value of outtake servo
                    setOuttakeState(SCORE_HIGH_SPEC);
                }
                break;
            case SCORE_HIGH_SPEC:
                // Code for attaching spec to rung
                setOuttakeState(RETRACT);
                break;
            case RETRACT:
                // wait for claw
                if (outtakeTimer.getElapsedTimeSeconds() > 1.3) {
                    bot.outtake_flat();
                    if (outtakeTimer.getElapsedTimeSeconds() > 2) {
                        bot.retractLift();
                        setOuttakeState(RobotConstants.OuttakeState.STOP);
                    }
                }
            case STOP:
                bot.outtake_flat();
                bot.retractLift();
                bot.claw_open();
                break;
        }
    }
}
