package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.LaunchArtifact;

import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name = "DECODE Auto - Blue Close", group = "Autonomous")
public class PedroBlueCloseAuto extends OpMode {

    private Follower follower;
    private Shooter shooter;
    private LaunchArtifact launchArtifact;
    private Timer pathTimer, opmodeTimer;

    private CRServo rightFeeder = null;
    private CRServo leftFeeder = null;
    private Servo leftStopper = null;
    private Servo rightStopper = null;
    private DcMotorEx intake = null;
    private int pathState;

    // ========== START POSE ==========
    private final Pose startPose = new Pose(15.500, 108.500, Math.toRadians(90));

    // ========== PATHS ==========
    private Path scorePreload;

    private PathChain grabPickup1, pickup1ForwardPath, scorePickup1;
    private PathChain grabPickup2, pickup2ForwardPath, scorePickup2;
    private PathChain grabPickup3, pickup3ForwardPath, scorePickup3;

    // ========== CONSTANTS ==========
    private static final double SHOOTER_SPINUP_TIME = .5;
    private static final double LAUNCH_TIME = 2.5;
    private static final int RPM_CLOSE = 3450;
    private static final int RPM_FAR = 4750;

    @Override
    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        leftFeeder = hardwareMap.get(CRServo.class, "left_feeder");
        rightFeeder = hardwareMap.get(CRServo.class, "right_feeder");

        leftStopper = hardwareMap.get(Servo.class, "left_stopper");
        rightStopper = hardwareMap.get(Servo.class, "right_stopper");

        this.leftFeeder.setDirection(CRServo.Direction.REVERSE);

        launchArtifact = new LaunchArtifact(leftFeeder, rightFeeder);

        follower = Constants.createFollower(hardwareMap);
        shooter = new Shooter(hardwareMap);

        intake = hardwareMap.get(DcMotorEx.class, "intake1");
        intake.setDirection(DcMotor.Direction.FORWARD);

        buildPaths();
        follower.setStartingPose(startPose);
    }

    @Override
    public void init_loop() {}

    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);
    }

    @Override
    public void loop() {
        follower.update();
        autonomousPathUpdate();
        launchArtifact.update();

        telemetry.addData("Path State", pathState);
        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("Heading", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.addData("Follower Busy", follower.isBusy());
        telemetry.addData("Time Elapsed", opmodeTimer.getElapsedTimeSeconds());
        telemetry.update();
    }

    @Override
    public void stop() {}

    // ========== PATH BUILDING ==========
    public void buildPaths() {

        // ===== PRELOAD =====
        scorePreload = new Path(
                new BezierLine(
                        new Pose(15.500, 108.500),
                        new Pose(63.500, 85.500)
                )
        );
        scorePreload.setLinearHeadingInterpolation(
                Math.toRadians(90),
                Math.toRadians(136)
        );

        // ===== PICKUP 1 =====
        grabPickup1 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(63.500, 85.500),
                        new Pose(46.000, 57.000)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(136), Math.toRadians(180))
                .build();

        pickup1ForwardPath = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(46.000, 57.000),
                        new Pose(10.500, 57.000)
                ))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        scorePickup1 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Pose(10.000, 58.000),
                        new Pose(42.581, 66.589),
                        new Pose(63.500, 85.500)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(136))
                .build();

        // ===== PICKUP 2 =====
        grabPickup2 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(63.500, 85.500),
                        new Pose(40.500, 83.400)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(136), Math.toRadians(180))
                .build();

        pickup2ForwardPath = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(40.500, 83.400),
                        new Pose(17.500, 83.400)
                ))
                .setTangentHeadingInterpolation()
                .build();

        scorePickup2 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(15.000, 83.400),
                        new Pose(63.500, 85.500)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(136))
                .build();

        // ===== PICKUP 3 =====
        grabPickup3 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(63.500, 85.500),
                                new Pose(64.791, 60.297),
                                new Pose(42.000, 35.000)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(136), Math.toRadians(180))
                .build();

        pickup3ForwardPath = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(42.000, 35.000),
                                new Pose(10.000, 35.000)
                        )
                )
                .setTangentHeadingInterpolation()
                .build();

        scorePickup3 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(10.000, 35.000),
                        new Pose(63.500, 85.500)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(136))
                .build();
    }

    // ========== FSM (UNCHANGED) ==========
    public void autonomousPathUpdate() {

        switch (pathState) {

            case 0:
                follower.followPath(scorePreload);
                spinupShooter(RPM_CLOSE);
                setPathState(1);
                break;

            case 1:
                if (!follower.isBusy()) {
                    spinupShooter(RPM_CLOSE);
                    setPathState(2);
                }
                break;

            case 2:
                if (pathTimer.getElapsedTimeSeconds() > SHOOTER_SPINUP_TIME) {
                    intakeSample();
                    launchSample();
                    setPathState(3);
                }
                break;

            case 3:
                if (pathTimer.getElapsedTimeSeconds() > LAUNCH_TIME) {
                    intakeSample();
                    follower.followPath(grabPickup1, true);
                    rightStopper.setPosition(0.55);
                    leftStopper.setPosition(0.55);
                    setPathState(4);
                }
                break;

            case 4:
                if (!follower.isBusy()) {
                    intakeSample();
                    follower.followPath(pickup1ForwardPath, true);
                    setPathState(5);
                }
                break;

            case 5:
                if (!follower.isBusy()) {
                    follower.followPath(scorePickup1, true);
                    setPathState(6);
                }
                break;

            case 6:
                if (!follower.isBusy()) {
                    spinupShooter(RPM_CLOSE);
                    setPathState(7);
                }
                break;

            case 7:
                intakeSample();
                prepareToLaunch();
                launchArtifact.fireLeftFeeder();
                launchArtifact.fireRightFeeder();
                setPathState(8);
                break;

            case 8:
                if (pathTimer.getElapsedTimeSeconds() > LAUNCH_TIME) {
                    stopShooter();
                    follower.followPath(grabPickup2, true);
                    rightStopper.setPosition(0.55);
                    leftStopper.setPosition(0.55);
                    setPathState(9);
                }
                break;

            case 9:
                if (!follower.isBusy()) {
                    intakeSample();
                    follower.followPath(pickup2ForwardPath, true);
                    setPathState(10);
                }
                break;

            case 10:
                if (!follower.isBusy()) {
                    stopIntake();
                    follower.followPath(scorePickup2, true);
                    setPathState(11);
                }
                break;

            case 11:
                if (!follower.isBusy()) {
                    spinupShooter(RPM_CLOSE);
                    setPathState(12);
                }
                break;

            case 12:
                if (pathTimer.getElapsedTimeSeconds() > .25) {
                    intakeSample();
                    launchSample();
                    setPathState(13);
                }
                break;

            case 13:
                if (pathTimer.getElapsedTimeSeconds() > LAUNCH_TIME) {
                    follower.followPath(grabPickup3, true);
                    rightStopper.setPosition(0.55);
                    leftStopper.setPosition(0.55);
                    setPathState(14);
                }
                break;

            case 14:
                if (!follower.isBusy()) {
                    intakeSample();
                    follower.followPath(pickup3ForwardPath, true);
                    setPathState(15);
                }
                break;

            case 15:
                if (!follower.isBusy()) {
                    stopIntake();
                    follower.followPath(scorePickup3, true);
                    setPathState(16);
                }
                break;

            case 16:
                if (!follower.isBusy()) {
                    spinupShooter(RPM_CLOSE); // ✅ FIXED (was FAR)
                    setPathState(17);
                }
                break;

            case 17:
                if (pathTimer.getElapsedTimeSeconds() > .5) {
                    intakeSample();
                    launchSample();
                    setPathState(18);
                }
                break;

            case 18:
                if (pathTimer.getElapsedTimeSeconds() > LAUNCH_TIME) {
                    setPathState(-1);
                }
                break;
        }
    }

    // ========== HELPERS ==========
    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    private void spinupShooter(int rpm) {
        shooter.setRPM(rpm);
    }

    private void prepareToLaunch() {
        rightStopper.setPosition(0.2);
        leftStopper.setPosition(0.6);
    }

    private void launchSample() {
        rightStopper.setPosition(0.2);
        leftStopper.setPosition(0.6);
        launchArtifact.fireLeftFeeder();
        launchArtifact.fireRightFeeder();
    }

    private void stopShooter() {
        rightStopper.setPosition(0.55);
        leftStopper.setPosition(0.55);
    }

    private void intakeSample() {
        intake.setPower(1.0);
    }

    private void stopIntake() {
        intake.setPower(0.0);
    }
}