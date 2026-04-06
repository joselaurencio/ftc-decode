package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.LaunchArtifact;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name = "DECODE Auto - FSM", group = "Autonomous")
public class ExampleAuto extends OpMode {
    private Follower follower;
    private Shooter shooter;
    private LaunchArtifact launchArtifact;
    private Timer pathTimer, opmodeTimer;

    private CRServo rightFeeder     = null;
    private CRServo leftFeeder      = null;
    private Servo leftStopper  = null;
    private Servo rightStopper = null;
    private DcMotorEx intake = null;
    private int pathState;

    // ========== POSES ==========
    private final Pose startPose = new Pose(121.55, 123.44, Math.toRadians(36));
    private final Pose scorePose = new Pose(80.5, 85.5, Math.toRadians(44));
    private final Pose scoreFarPose = new Pose(85, 13, Math.toRadians(73));

    private final Pose pickup1Pose = new Pose(98, 60, Math.toRadians(0));
    private final Pose pickup2Pose = new Pose(103.5, 83.4, Math.toRadians(0));
    private final Pose pickup3Pose = new Pose(96, 35, Math.toRadians(0));

    // Forward poses (20 inches forward)
    private final Pose pickup1Forward = new Pose(103.5 + 22, 60, Math.toRadians(0));
    private final Pose pickup2Forward = new Pose(103.5 + 20, 83.4, Math.toRadians(0));
    private final Pose pickup3Forward = new Pose(103.5 + 22, 35, Math.toRadians(0));

    // ========== PATHS ==========
    private Path scorePreload;

    private PathChain grabPickup1, pickup1ForwardPath, scorePickup1;
    private PathChain grabPickup2, pickup2ForwardPath, scorePickup2;
    private PathChain grabPickup3, pickup3ForwardPath, scorePickup3;

    // ========== CONSTANTS ==========
    private static final double SHOOTER_SPINUP_TIME = .5;
    private static final double LAUNCH_TIME = 2.5;
    private static final int RPM_CLOSE = 3300;
    private static final int RPM_FAR = 4550;

    @Override
    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        leftFeeder = hardwareMap.get(CRServo.class, "left_feeder");
        rightFeeder = hardwareMap.get(CRServo.class, "right_feeder");

        leftStopper  = hardwareMap.get(Servo.class, "left_stopper");
        rightStopper = hardwareMap.get(Servo.class, "right_stopper");

        this.leftFeeder.setDirection(CRServo.Direction.REVERSE);

        launchArtifact = new LaunchArtifact(leftFeeder, rightFeeder);

        // Initialize follower from Constants
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

        // Telemetry for debugging
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

        scorePreload = new Path(new BezierLine(startPose, scorePose));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading());

        grabPickup1 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, pickup1Pose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup1Pose.getHeading())
                .build();

        pickup1ForwardPath = follower.pathBuilder()
                .addPath(new BezierLine(pickup1Pose, pickup1Forward))
                .setLinearHeadingInterpolation(pickup1Pose.getHeading(), pickup1Forward.getHeading())
                .build();

        scorePickup1 = follower.pathBuilder()
                .addPath(new BezierLine(pickup1Forward, scorePose))
                .setLinearHeadingInterpolation(pickup1Forward.getHeading(), scorePose.getHeading())
                .build();

        grabPickup2 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, pickup2Pose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup2Pose.getHeading())
                .build();

        pickup2ForwardPath = follower.pathBuilder()
                .addPath(new BezierLine(pickup2Pose, pickup2Forward))
                .setLinearHeadingInterpolation(pickup2Pose.getHeading(), pickup2Forward.getHeading())
                .build();

        scorePickup2 = follower.pathBuilder()
                .addPath(new BezierLine(pickup2Forward, scorePose))
                .setLinearHeadingInterpolation(pickup2Forward.getHeading(), scorePose.getHeading())
                .build();

        grabPickup3 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, pickup3Pose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup3Pose.getHeading())
                .build();

        pickup3ForwardPath = follower.pathBuilder()
                .addPath(new BezierLine(pickup3Pose, pickup3Forward))
                .setLinearHeadingInterpolation(pickup3Pose.getHeading(), pickup3Forward.getHeading())
                .build();

        scorePickup3 = follower.pathBuilder()
                .addPath(new BezierLine(pickup3Forward, scoreFarPose))
                .setLinearHeadingInterpolation(pickup3Forward.getHeading(), scoreFarPose.getHeading())
                .build();
    }

    // ========== FSM ==========
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

            // PICKUP 1 FIX
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
                // Wait for launch to complete before moving
                if (pathTimer.getElapsedTimeSeconds() > LAUNCH_TIME) {
                    stopShooter();
                    follower.followPath(grabPickup2, true);
                    rightStopper.setPosition(0.55);
                    leftStopper.setPosition(0.55);
                    setPathState(9);
                }
                break;

            case 9:
                // OLD case 9 moves here
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

            // PICKUP 3 FIX
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
                    spinupShooter(RPM_FAR);
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

    // ========== HELPER METHODS ==========
    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    // TODO: Connect these to your actual subsystems
    private void spinupShooter(int rpm) {
        // Call your shooter subsystem to spinup flywheels
        // Example: shooter.setRPM(rpm);
        shooter.setRPM(rpm);
        telemetry.addData("Shooter", "Spinning up to " + rpm + " RPM");
    }
    private void prepareToLaunch() {
        rightStopper.setPosition(0.2);
        leftStopper.setPosition(0.6);
    }
    private void launchSample() {
        // Use LaunchArtifact to fire the feeder
        rightStopper.setPosition(0.2);
        leftStopper.setPosition(0.6);
        launchArtifact.fireLeftFeeder();
        launchArtifact.fireRightFeeder();
        telemetry.addData("Shooter", "Launching!");
    }

    private void stopShooter() {
        // Stop flywheel and feeder
        rightStopper.setPosition(0.55);
        leftStopper.setPosition(0.55);
        telemetry.addData("Shooter", "Stopped");
    }

    private void intakeSample() {
        intake.setPower(1.0);  // Run intake at full speed
        telemetry.addData("Intake Velocity", intake.getVelocity());  // ticks/sec
        telemetry.addData("Intake RPM", intake.getVelocity() * 60 / 28);
        telemetry.addData("Intake", "Running");
    }

    private void stopIntake() {
        intake.setPower(0.0);  // Stop intake
        telemetry.addData("Intake", "Stopped");
    }
}