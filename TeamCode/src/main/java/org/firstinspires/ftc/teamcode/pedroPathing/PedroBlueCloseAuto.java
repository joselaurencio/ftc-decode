package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.LaunchArtifact;
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

    private CRServo rightFeeder     = null;
    private CRServo leftFeeder      = null;
    private Servo leftStopper       = null;
    private Servo rightStopper      = null;
    private DcMotorEx intake        = null;
    private int pathState;

    // ========== POSES ==========
    private final Pose startPose    = new Pose(19.749, 121.783, Math.toRadians(144));
    private final Pose scorePose    = new Pose(56.400, 87.000, Math.toRadians(131));

    private final Pose pickup1Pose      = new Pose(42.000, 60.000, Math.toRadians(180));
    private final Pose pickup1Forward   = new Pose(10.000, 60.000, Math.toRadians(180));

    private final Pose pickup2Pose      = new Pose(42.000, 84.000, Math.toRadians(180));
    private final Pose pickup2Forward   = new Pose(18.000, 84.000, Math.toRadians(180));

    private final Pose openChamberPose  = new Pose(10.834, 60.480, Math.toRadians(144));

    // ========== PATHS ==========
    private Path scorePreload;

    private PathChain grabPickup1, pickup1ForwardPath, scorePickup1;
    private PathChain grabPickup2, pickup2ForwardPath, scorePickup2;
    private PathChain openChamber;

    // ========== CONSTANTS ==========
    private static final double SHOOTER_SPINUP_TIME = 0.5;
    private static final double LAUNCH_TIME         = 2.5;
    private static final int    RPM_CLOSE           = 3300;
    private static final int    RPM_FAR             = 4550;

    @Override
    public void init() {
        pathTimer    = new Timer();
        opmodeTimer  = new Timer();
        opmodeTimer.resetTimer();

        leftFeeder   = hardwareMap.get(CRServo.class, "left_feeder");
        rightFeeder  = hardwareMap.get(CRServo.class, "right_feeder");
        leftStopper  = hardwareMap.get(Servo.class,   "left_stopper");
        rightStopper = hardwareMap.get(Servo.class,   "right_stopper");

        this.leftFeeder.setDirection(CRServo.Direction.REVERSE);

        launchArtifact = new LaunchArtifact(leftFeeder, rightFeeder);

        follower = Constants.createFollower(hardwareMap);
        shooter  = new Shooter(hardwareMap);

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

        telemetry.addData("Path State",     pathState);
        telemetry.addData("X",              follower.getPose().getX());
        telemetry.addData("Y",              follower.getPose().getY());
        telemetry.addData("Heading",        Math.toDegrees(follower.getPose().getHeading()));
        telemetry.addData("Follower Busy",  follower.isBusy());
        telemetry.addData("Time Elapsed",   opmodeTimer.getElapsedTimeSeconds());
        telemetry.update();
    }

    @Override
    public void stop() {}

    // ========== PATH BUILDING ==========
    public void buildPaths() {

        // Preload → score
        scorePreload = new Path(new BezierLine(startPose, scorePose));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading());

        // Score → grab pickup 1
        grabPickup1 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, pickup1Pose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup1Pose.getHeading())
                .build();

        // Pickup 1 → push forward
        pickup1ForwardPath = follower.pathBuilder()
                .addPath(new BezierLine(pickup1Pose, pickup1Forward))
                .setLinearHeadingInterpolation(pickup1Pose.getHeading(), pickup1Forward.getHeading())
                .build();

        // Pickup 1 forward → score
        scorePickup1 = follower.pathBuilder()
                .addPath(new BezierLine(pickup1Forward, scorePose))
                .setLinearHeadingInterpolation(pickup1Forward.getHeading(), scorePose.getHeading())
                .build();

        // Score → grab pickup 2
        grabPickup2 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, pickup2Pose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup2Pose.getHeading())
                .build();

        // Pickup 2 → push forward
        pickup2ForwardPath = follower.pathBuilder()
                .addPath(new BezierLine(pickup2Pose, pickup2Forward))
                .setTangentHeadingInterpolation()
                .build();

        // Pickup 2 forward → score
        scorePickup2 = follower.pathBuilder()
                .addPath(new BezierLine(pickup2Forward, scorePose))
                .setLinearHeadingInterpolation(pickup2Forward.getHeading(), scorePose.getHeading())
                .build();

        // Score → open chamber (replaces pickup 3 cycle)
        openChamber = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, openChamberPose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), openChamberPose.getHeading())
                .build();
    }

    // ========== FSM ==========
    public void autonomousPathUpdate() {
        switch (pathState) {

            case 0:
                // Start driving to score, spinup shooter, intake OFF while driving to score
                follower.followPath(scorePreload);
                spinupShooter(RPM_CLOSE);
                stopIntake();
                setPathState(1);
                break;

            case 1:
                // Wait until at score pose
                if (!follower.isBusy()) {
                    spinupShooter(RPM_CLOSE);
                    setPathState(2);
                }
                break;

            case 2:
                // Wait for shooter spinup then launch preload, intake ON for launch assist
                if (pathTimer.getElapsedTimeSeconds() > SHOOTER_SPINUP_TIME) {
                    intakeSample();
                    launchSample();
                    setPathState(3);
                }
                break;

            case 3:
                // After launch, drive to pickup 1, stoppers open, intake OFF while driving
                if (pathTimer.getElapsedTimeSeconds() > LAUNCH_TIME) {
                    stopIntake();
                    follower.followPath(grabPickup1, true);
                    rightStopper.setPosition(0.55);
                    leftStopper.setPosition(0.55);
                    setPathState(4);
                }
                break;

            case 4:
                // At pickup 1 position, intake ON, push forward
                if (!follower.isBusy()) {
                    intakeSample();
                    follower.followPath(pickup1ForwardPath, true);
                    setPathState(5);
                }
                break;

            case 5:
                // Done pushing forward, intake OFF, drive back to score
                if (!follower.isBusy()) {
                    stopIntake();
                    follower.followPath(scorePickup1, true);
                    setPathState(6);
                }
                break;

            case 6:
                // At score pose, spinup shooter
                if (!follower.isBusy()) {
                    spinupShooter(RPM_CLOSE);
                    setPathState(7);
                }
                break;

            case 7:
                // Launch pickup 1 sample, intake ON for launch assist
                if (pathTimer.getElapsedTimeSeconds() > SHOOTER_SPINUP_TIME) {
                    intakeSample();
                    prepareToLaunch();
                    launchArtifact.fireLeftFeeder();
                    launchArtifact.fireRightFeeder();
                    setPathState(8);
                }
                break;

            case 8:
                // Wait for launch, then drive to pickup 2, intake OFF while driving
                if (pathTimer.getElapsedTimeSeconds() > LAUNCH_TIME) {
                    stopShooter();
                    stopIntake();
                    follower.followPath(grabPickup2, true);
                    rightStopper.setPosition(0.55);
                    leftStopper.setPosition(0.55);
                    setPathState(9);
                }
                break;

            case 9:
                // At pickup 2 position, intake ON, push forward
                if (!follower.isBusy()) {
                    intakeSample();
                    follower.followPath(pickup2ForwardPath, true);
                    setPathState(10);
                }
                break;

            case 10:
                // Done pushing forward, intake OFF, drive back to score
                if (!follower.isBusy()) {
                    stopIntake();
                    follower.followPath(scorePickup2, true);
                    setPathState(11);
                }
                break;

            case 11:
                // At score pose, spinup shooter
                if (!follower.isBusy()) {
                    spinupShooter(RPM_CLOSE);
                    setPathState(12);
                }
                break;

            case 12:
                // Launch pickup 2 sample, intake ON for launch assist
                if (pathTimer.getElapsedTimeSeconds() > SHOOTER_SPINUP_TIME) {
                    intakeSample();
                    launchSample();
                    setPathState(13);
                }
                break;

            case 13:
                // After launch, drive to open chamber, intake OFF
                if (pathTimer.getElapsedTimeSeconds() > LAUNCH_TIME) {
                    stopIntake();
                    follower.followPath(openChamber, true);
                    setPathState(14);
                }
                break;

            case 14:
                // Arrived at open chamber — placeholder for future logic
                if (!follower.isBusy()) {
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

    private void spinupShooter(int rpm) {
        shooter.setRPM(rpm);
        telemetry.addData("Shooter", "Spinning up to " + rpm + " RPM");
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
        telemetry.addData("Shooter", "Launching!");
    }

    private void stopShooter() {
        rightStopper.setPosition(0.55);
        leftStopper.setPosition(0.55);
        telemetry.addData("Shooter", "Stopped");
    }

    private void intakeSample() {
        intake.setPower(1.0);
        telemetry.addData("Intake Velocity", intake.getVelocity());
        telemetry.addData("Intake RPM",      intake.getVelocity() * 60 / 28);
        telemetry.addData("Intake",          "Running");
    }

    private void stopIntake() {
        intake.setPower(0.0);
        telemetry.addData("Intake", "Stopped");
    }
}