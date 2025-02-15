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
import static org.firstinspires.ftc.teamcode.autonomous.FieldConstants.blueAllianceBlueRightSpike;
import static org.firstinspires.ftc.teamcode.autonomous.FieldConstants.blueAllianceNeutralCenterSpike;
import static org.firstinspires.ftc.teamcode.autonomous.FieldConstants.blueAllianceNeutralLeftSpike;
import static org.firstinspires.ftc.teamcode.autonomous.FieldConstants.blueAllianceNeutralRightSpike;
import static org.firstinspires.ftc.teamcode.autonomous.FieldConstants.blueAlliancePreloadSpec;
import static org.firstinspires.ftc.teamcode.autonomous.FieldConstants.blueAllianceSample1;
import static org.firstinspires.ftc.teamcode.autonomous.FieldConstants.blueAllianceSample2;
import static org.firstinspires.ftc.teamcode.autonomous.FieldConstants.blueAllianceSample3;
import static org.firstinspires.ftc.teamcode.autonomous.FieldConstants.blueAllianceSpec;
import static org.firstinspires.ftc.teamcode.autonomous.FieldConstants.blueAllianceSpec1;
import static org.firstinspires.ftc.teamcode.autonomous.FieldConstants.blueAllianceSpec1turn;
import static org.firstinspires.ftc.teamcode.autonomous.FieldConstants.blueAllianceSpec2;
import static org.firstinspires.ftc.teamcode.autonomous.FieldConstants.blueAllianceSpec3;
import static org.firstinspires.ftc.teamcode.autonomous.FieldConstants.blueAllianceSpecPickup;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.PathBuilder;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.MecanumTrain;
import org.firstinspires.ftc.teamcode.RobotConstants.IntakeState;
import org.firstinspires.ftc.teamcode.RobotConstants.LiftState;
@Disabled
@Autonomous(name = "Basket No Specimen Preload + 3", group = "Basket")
public class auto extends OpMode {

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

    Path preloadBasketScore, rightSampleScore, centerSampleScore, leftSampleScore, parkPath;

    PathChain rightSampleCycleChain, centerSampleCycleChain, leftSampleCycleChain;
    private Path preloadSpecScore;
    private Path Spec1;
    private BezierCurve Sample1PickUp;
    private PathChain SampleCycleChain;
    private PathBuilder SpecimenScore;
    private PathBuilder Specimen1Score;
    private PathBuilder Specimen2Score;
    private PathBuilder Specimen3Score;

    @Override
    public void init() {
        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        intakeTimer = new Timer();
        pathTimer = new Timer();
        liftTimer = new Timer();
        opmodeTimer = new ElapsedTime();
        fullTimer = new Timer();

        bot = new MecanumTrain(hardwareMap, opmodeTimer);
//        bot.follower.setStartingPose(blueAllianceBasketStart);
//        bot.follower.setMaxPower(0.9);
//

        bot.setIntakeServo("off");
        bot.setIntakePivot("in");
        bot.setHorizontalExtension("in");
    }

    @Override
    public void start() {
        opmodeTimer.reset();
        fullTimer.resetTimer();
//        buildPaths();
        setIntakeState(INTAKE_INIT);
        setLiftState(LIFT_INIT);
//        setPathState(1);
    }

    @Override
    public void loop() {
//        bot.follower.update();

//        autonomousPathUpdate();
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
//        telemetry.addData("Position", bot.follower.getPose());
        telemetry.update();
    }

    private void buildPaths() {

        preloadSpecScore = new Path(new BezierLine(
                new Point(startPose),
                blueAlliancePreloadSpec
        ));

        SampleCycleChain = new PathBuilder()
                .addPath(new BezierCurve(
                        blueAllianceSample1,
                        new Point(15.6, 50.9)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(315), Math.toRadians(215))
                .addPath(new BezierCurve(
                        blueAllianceSample2
                ))
                .setConstantHeadingInterpolation(Math.toRadians(65)) // fix
                .addPath(new BezierCurve(
                        blueAllianceSample3
                ))
                .setConstantHeadingInterpolation(Math.toRadians(65)) // fix
                .build();

        Specimen1Score = new PathBuilder()
                // specimen picup to score spec 1
                .addPath(new Path(new BezierLine(
                            blueAllianceSpecPickup,
                            blueAllianceSpec1)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(315), Math.toRadians(0))
                .addPath(new Path(new BezierLine(
                                blueAllianceSpec1,
                                blueAllianceSpecPickup
                                )))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(315));
        Specimen2Score = new PathBuilder()
                .addPath(new Path(new BezierLine(
                        blueAllianceSpecPickup,
                        blueAllianceSpec2
                        )))
                .setLinearHeadingInterpolation(Math.toRadians(315), Math.toRadians(0))
                .addPath(new Path(new BezierLine(
                        blueAllianceSpec2,
                        blueAllianceSpecPickup
                        )))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(315));
        Specimen3Score = new PathBuilder()

                .addPath(new Path(new BezierLine(
                        blueAllianceSpecPickup,
                        blueAllianceSpec3
                )));
    }


    private void setIntakeState(IntakeState iState) {
        intakeState = iState;
        intakeTimer.resetTimer();
        intakeStateUpdate();
    }

    private void setLiftState(LiftState lState) {
        liftState = lState;
        liftTimer.resetTimer();
        outtakeStateUpdate();
    }

    int count = 0; // for only spitting sample 3 times
    private void intakeStateUpdate() {
        switch (intakeState) {
            case INTAKE_INIT:
                break;
            case INTAKE_FLIP_OUT:
                // flip out the intake
                setIntakeState(INTAKE_SPIN);
                break;
            case INTAKE_SPIN:
                // intake a sample
                if (true) { // replace true with sample detect code
                    setIntakeState(INTAKE_SAMPLE_IN);
                }
                break;
            case INTAKE_SAMPLE_OUT:
                // rotate the robot however many degrees
                // spit out sample
                count++;
                if(count==3) {
                    setIntakeState(INTAKE_STOP);
                }
                setIntakeState(INTAKE_SPIN);
                break;
            case INTAKE_STOP:
                // stop conditions
                break;
        }
    }

    // pls change state names they kept throwing errors
    private void outtakeStateUpdate() {
        switch (liftState) {
            case LIFT_INIT:
                //
                break;
            case LIFT_START:
                setLiftState(LIFT_EXTEND_LOW);
                break;
            case LIFT_EXTEND_LOW: // LIFT_PICK_SPEC
                // move to spec pickup location
                // code to pick up spec with outtake
                if(true) { // check that spec is picked up
                    setLiftState(LIFT_EXTEND_HIGH);
                }
            case LIFT_EXTEND_HIGH: // LIFT_SCORE_SPEC
                // move to submersible to score spec
                // code to score spec
                setLiftState(LIFT_RETRACT);
                break;
            case LIFT_RETRACT:
                // retract slides
                setLiftState(LIFT_EXTEND_LOW);
                break;
            case LIFT_STOP:
                // needs to come to this state when time runs out
                break;
        }
    }
}
