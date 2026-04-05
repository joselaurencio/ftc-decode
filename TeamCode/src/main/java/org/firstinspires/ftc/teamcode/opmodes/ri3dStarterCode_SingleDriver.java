package org.firstinspires.ftc.teamcode.opmodes;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.vision.LimelightVision;
import org.firstinspires.ftc.teamcode.math.ShooterModel;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;

@TeleOp(name = "DECODE Ri3D SINGLE DRIVER", group = "StarterBot")
//@Disabled
public class ri3dStarterCode_SingleDriver extends OpMode {

    // ================= AUTO ALIGN PD =================
    private double kP_align = 0.12;   // raised from 0.08 — tune up from here
    private double kD_align = 0.003;
    private double previousLeftAlignError  = 0;  // separate per shooter
    private double previousRightAlignError = 0;

    private final double ALIGN_DEADBAND  = 0.2;
    private final double MAX_ALIGN_POWER = 0.7;
    // ==================================================

    // ================= DYNAMIC OFFSET TUNING =================
    private final double BASE_OFFSET     = -3.0;
    private final double DISTANCE_FACTOR =  0.002;
    private final double SHOOTER_OFFSET  =  0.1;
    // =========================================================

    final double FEED_TIME_SECONDS = 1.5;
    final double STOP_SPEED        = 0.0;
    final double FULL_SPEED        = 1.0;

    final double RPM_CLOSE_TARGET = 2000;
    final double RPM_CLOSE_MIN    = 1900;

    final double LAUNCHER_CLOSE_TARGET_VELOCITY = RPM_CLOSE_TARGET * 28 / 60.0;
    final double LAUNCHER_CLOSE_MIN_VELOCITY    = RPM_CLOSE_MIN    * 28 / 60.0;

    final double RPM_FAR_TARGET = 3200;
    final double RPM_FAR_MIN    = 3000;

    final double LAUNCHER_FAR_TARGET_VELOCITY = RPM_FAR_TARGET * 28 / 60.0;
    final double LAUNCHER_FAR_MIN_VELOCITY    = RPM_FAR_MIN    * 28 / 60.0;

    double launcherTarget = LAUNCHER_CLOSE_TARGET_VELOCITY;
    double launcherMin    = LAUNCHER_CLOSE_MIN_VELOCITY;

    final double LEFT_POSITION  = 0.5;
    final double RIGHT_POSITION = 0;

    private boolean lastBack  = false;
    private boolean lastY = false;
    private boolean lastGP2RB = false;
    private boolean lastGP2LB = false;

    // ===== Hardware =====
    private DcMotor   leftFrontDrive  = null;
    private DcMotor   rightFrontDrive = null;
    private DcMotor   leftBackDrive   = null;
    private DcMotor   rightBackDrive  = null;
    private DcMotorEx leftLauncher    = null;
    private DcMotorEx rightLauncher   = null;
    private DcMotorEx intake1         = null;
    private CRServo   leftFeeder      = null;
    private CRServo   rightFeeder     = null;
    private Servo     diverter        = null;
    private Servo     leftStopper     = null;
    private Servo     rightStopper    = null;

    private LimelightVision limelight;

    ElapsedTime leftFeederTimer  = new ElapsedTime();
    ElapsedTime rightFeederTimer = new ElapsedTime();

    // ===== State Enums =====
    private enum LaunchState { IDLE, ALIGN, SPIN_UP, LAUNCH, LAUNCHING }
    private LaunchState leftLaunchState;
    private LaunchState rightLaunchState;

    private enum ManualLaunchState { IDLE, SPIN_UP, LAUNCH, LAUNCHING }
    private ManualLaunchState manualLeftState  = ManualLaunchState.IDLE;
    private ManualLaunchState manualRightState = ManualLaunchState.IDLE;

    private double manualLeftTarget  = 0;
    private double manualRightTarget = 0;

    private enum DiverterDirection { LEFT, RIGHT }
    private DiverterDirection diverterDirection = DiverterDirection.LEFT;

    private enum IntakeState { ON, OFF, REVERSE }
    private IntakeState intakeState = IntakeState.OFF;

    private enum ShooterMode { MANUAL, VISION }
    private ShooterMode shooterMode = ShooterMode.VISION;

    private enum LauncherDistance { CLOSE, FAR }
    private LauncherDistance launcherDistance = LauncherDistance.CLOSE;

    // ===== Intake constants =====
    private static final double INTAKE_RPM             = 6000;
    private static final double INTAKE_TICKS_PER_REV   = 28.0;
    private static final double INTAKE_TARGET_VELOCITY  = INTAKE_RPM * INTAKE_TICKS_PER_REV / 60.0;

    // ===== BALL COUNTER =====
    private int ballCount = 0;

    // How far below target velocity counts as a "ball detected" dip.
// Tune this — start around 30-40% of target velocity.
    private static final double BALL_DIP_THRESHOLD  = INTAKE_TARGET_VELOCITY * 0.35;

    // Once dip is detected, ignore further dips for this many ms
    // (prevents one ball from being counted twice as RPM bounces)
    private static final long   BALL_DEBOUNCE_MS    = 400;

    private boolean ballDipActive    = false;
    private long    lastBallDetected = 0;
    double leftFrontPower;
    double rightFrontPower;
    double leftBackPower;
    double rightBackPower;

    // =========================================================
    // INIT
    // =========================================================
    @Override
    public void init() {
        leftLaunchState  = LaunchState.IDLE;
        rightLaunchState = LaunchState.IDLE;

        leftFrontDrive  = hardwareMap.get(DcMotor.class,   "left_front_drive");
        rightFrontDrive = hardwareMap.get(DcMotor.class,   "right_front_drive");
        leftBackDrive   = hardwareMap.get(DcMotor.class,   "left_back_drive");
        rightBackDrive  = hardwareMap.get(DcMotor.class,   "right_back_drive");
        leftLauncher    = hardwareMap.get(DcMotorEx.class, "left_launcher");
        rightLauncher   = hardwareMap.get(DcMotorEx.class, "right_launcher");
        intake1         = hardwareMap.get(DcMotorEx.class, "intake1");
        leftFeeder      = hardwareMap.get(CRServo.class,   "left_feeder");
        rightFeeder     = hardwareMap.get(CRServo.class,   "right_feeder");
        diverter        = hardwareMap.get(Servo.class,     "diverter");
        leftStopper     = hardwareMap.get(Servo.class,     "left_stopper");
        rightStopper    = hardwareMap.get(Servo.class,     "right_stopper");

        leftStopper.setPosition(0.7);
        rightStopper.setPosition(0.7);

        limelight = new LimelightVision(hardwareMap);

        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        leftLauncher.setDirection(DcMotorSimple.Direction.FORWARD);
        rightLauncher.setDirection(DcMotorSimple.Direction.FORWARD);

        intake1.setDirection(DcMotorSimple.Direction.FORWARD);

        leftLauncher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightLauncher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intake1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftFrontDrive.setZeroPowerBehavior(BRAKE);
        rightFrontDrive.setZeroPowerBehavior(BRAKE);
        leftBackDrive.setZeroPowerBehavior(BRAKE);
        rightBackDrive.setZeroPowerBehavior(BRAKE);
        leftLauncher.setZeroPowerBehavior(BRAKE);
        rightLauncher.setZeroPowerBehavior(BRAKE);
        intake1.setZeroPowerBehavior(BRAKE);

        leftFeeder.setPower(STOP_SPEED);
        rightFeeder.setPower(STOP_SPEED);

        intake1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,
                new PIDFCoefficients(10, 0, 0, 1));
        leftLauncher.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,
                new PIDFCoefficients(300, 0, 0, 10));
        rightLauncher.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,
                new PIDFCoefficients(300, 0, 0, 10));

        rightFeeder.setDirection(DcMotorSimple.Direction.FORWARD);
        leftFeeder.setDirection(DcMotorSimple.Direction.REVERSE);

        telemetry.addData("Status", "Initialized");
    }

    @Override
    public void init_loop() {}

    @Override
    public void start() {}

    // =========================================================
    // MAIN LOOP
    // =========================================================
    @Override
    public void loop() {

        limelight.update();

        // ===== Vision Mode: continuously update launcher velocity target =====
        if (shooterMode == ShooterMode.VISION && limelight.hasTarget()) {
            double distanceCm = limelight.getDistanceFromArea();

            double leftRPM  = ShooterModel.distanceToRPM(distanceCm, true);
            double rightRPM = ShooterModel.distanceToRPM(distanceCm, false);

            double selectedRPM;
            if (leftLaunchState != LaunchState.IDLE) {
                selectedRPM = leftRPM;
            } else if (rightLaunchState != LaunchState.IDLE) {
                selectedRPM = rightRPM;
            } else {
                selectedRPM = leftRPM;
            }

            launcherTarget = selectedRPM * 28.0 / 60.0;
            launcherMin    = launcherTarget * 0.95;
        }

        // ===== Toggle Vision/Manual mode (BACK) =====
        if (gamepad1.back && !lastBack) {
            shooterMode = (shooterMode == ShooterMode.MANUAL)
                    ? ShooterMode.VISION
                    : ShooterMode.MANUAL;
        }
        lastBack = gamepad1.back;

        // ===== Driver input =====
        double forward = -gamepad1.left_stick_y;
        double strafe  =  gamepad1.left_stick_x;
        double rotate  =  gamepad1.right_stick_x;

        // ===== Manual auto-align override (hold right trigger) =====
        // This is the continuous hold-to-align — separate from the
        // FSM align that triggers on bumper press.
        boolean autoAlignActive = gamepad1.right_trigger > 0.5;

        if (autoAlignActive && limelight.hasTarget()) {
            double currentYaw = limelight.getTx();
            double error      = -currentYaw;
            double derivative = error - previousLeftAlignError; // reuse left for manual align
            double output     = (error * kP_align) + (derivative * kD_align);

            if (Math.abs(error) < ALIGN_DEADBAND) output = 0;
            output = Math.max(-MAX_ALIGN_POWER, Math.min(MAX_ALIGN_POWER, output));

            rotate                 = output;
            previousLeftAlignError = error;
        } else if (!autoAlignActive) {
            previousLeftAlignError = 0;
        }

        mecanumDrive(forward, strafe, rotate);
        updateBallCounter();

        // ===== Manual flywheel spin-up (hold Y) =====
        if (gamepad1.y) {
            leftLauncher.setVelocity(launcherTarget);
            rightLauncher.setVelocity(launcherTarget);
        }

        // ===== Ball counter reset (Y press) =====
        if (gamepad1.y && !lastY) {
            ballCount = 0;
        }
        lastY = gamepad1.y;

        // ===== Diverter toggle (D-Pad Down) =====
        if (gamepad1.dpadDownWasPressed()) {
            switch (diverterDirection) {
                case LEFT:
                    diverterDirection = DiverterDirection.RIGHT;
                    diverter.setPosition(RIGHT_POSITION);
                    break;
                case RIGHT:
                    diverterDirection = DiverterDirection.LEFT;
                    diverter.setPosition(LEFT_POSITION);
                    break;
            }
        }

        // ===== Intake toggle =====
        // A = toggle intake IN
        // X = toggle intake REVERSE
        if (gamepad1.aWasPressed()) {
            if (intakeState == IntakeState.ON) {
                intakeState = IntakeState.OFF;
                setIntakePower(0);
            } else {
                intakeState = IntakeState.ON;
                setIntakePower(1);
            }
        }

        if (gamepad1.xWasPressed()) {
            if (intakeState == IntakeState.REVERSE) {
                intakeState = IntakeState.OFF;
                setIntakePower(0);
            } else {
                intakeState = IntakeState.REVERSE;
                setIntakePower(-1);
            }
        }

        // ===== Distance preset toggle (D-Pad Up) =====
        if (gamepad1.dpadUpWasPressed()) {
            switch (launcherDistance) {
                case CLOSE:
                    launcherDistance = LauncherDistance.FAR;
                    launcherTarget   = LAUNCHER_FAR_TARGET_VELOCITY;
                    launcherMin      = LAUNCHER_FAR_MIN_VELOCITY;
                    break;
                case FAR:
                    launcherDistance = LauncherDistance.CLOSE;
                    launcherTarget   = LAUNCHER_CLOSE_TARGET_VELOCITY;
                    launcherMin      = LAUNCHER_CLOSE_MIN_VELOCITY;
                    break;
            }
        }

        // ===== Vision launch state machines (GP1 bumpers) =====
        launchLeft(gamepad1.leftBumperWasPressed());
        launchRight(gamepad1.rightBumperWasPressed());

        // ===== Manual (no-align) launch state machines (GP2 bumpers) =====
        boolean gp2rb = gamepad2.right_bumper;
        boolean gp2lb = gamepad2.left_bumper;
        manualLaunchLeft( gp2rb && !lastGP2RB);
        manualLaunchRight(gp2lb && !lastGP2LB);
        lastGP2RB = gp2rb;
        lastGP2LB = gp2lb;

        // ===== Telemetry =====
        final double leftRPM  = leftLauncher.getVelocity()  * 60 / 28;
        final double rightRPM = rightLauncher.getVelocity() * 60 / 28;

        telemetry.addData("Shooter Mode",       shooterMode);
        telemetry.addData("Launcher Distance",  launcherDistance);
        telemetry.addData("Auto Align Active",  autoAlignActive);
        telemetry.addData("Left velocity",      leftLauncher.getVelocity());
        telemetry.addData("launcherMin",        launcherMin);
        telemetry.addData("Left Launch State",  leftLaunchState);
        telemetry.addData("Right Launch State", rightLaunchState);
        telemetry.addData("Manual Left State",  manualLeftState);
        telemetry.addData("Manual Right State", manualRightState);
        telemetry.addData("Left RPM",           leftRPM);
        telemetry.addData("Right RPM",          rightRPM);
        telemetry.addData("Vision Target TPS",  launcherTarget);
        telemetry.addData("Intake State",       intakeState);

        if (limelight.hasTarget()) {
            telemetry.addData("Limelight tx",    limelight.getTx());
            telemetry.addData("Distance (cm)",   limelight.getDistanceFromArea());
            telemetry.addData("L targetOffset",  getTargetOffset(true));
            telemetry.addData("R targetOffset",  getTargetOffset(false));
        }
        telemetry.addData("Balls Counted",     ballCount);
        telemetry.addData("Intake Velocity",   intake1.getVelocity());
        telemetry.addData("Intake Target",     INTAKE_TARGET_VELOCITY);
        telemetry.addData("Velocity Drop",     INTAKE_TARGET_VELOCITY - intake1.getVelocity());

        telemetry.addData("Manual L target TPS", manualLeftTarget);
        telemetry.addData("Manual R target TPS", manualRightTarget);
    }

    // =========================================================
    // INTAKE HELPER
    // =========================================================
    private void setIntakePower(double power) {
        intake1.setVelocity(power * INTAKE_TARGET_VELOCITY);
    }
    private void updateBallCounter() {
        // Don't count during a shot — feeder noise can fake a dip
        if (leftLaunchState  == LaunchState.LAUNCHING ||
                rightLaunchState == LaunchState.LAUNCHING) return;
        if (intakeState != IntakeState.ON) return;
        // Only count when intake is actually running inward


        double currentVelocity = intake1.getVelocity();
        double velocityDrop    = INTAKE_TARGET_VELOCITY - currentVelocity;
        long   now             = System.currentTimeMillis();

        // Dip detected — velocity dropped significantly below target
        if (velocityDrop > BALL_DIP_THRESHOLD && !ballDipActive
                && (now - lastBallDetected) > BALL_DEBOUNCE_MS) {
            ballDipActive = true;
        }

        // Recovery — velocity came back up, ball has passed through
        if (ballDipActive && velocityDrop < BALL_DIP_THRESHOLD * 0.5) {
            ballCount++;
            ballDipActive     = false;
            lastBallDetected  = now;
        }
    }

    // =========================================================
    // MECANUM DRIVE
    // =========================================================
    void mecanumDrive(double forward, double strafe, double rotate) {
        double denominator = Math.max(Math.abs(forward) + Math.abs(strafe) + Math.abs(rotate), 1);

        leftFrontPower  = (forward + strafe + rotate) / denominator;
        rightFrontPower = (forward - strafe - rotate) / denominator;
        leftBackPower   = (forward - strafe + rotate) / denominator;
        rightBackPower  = (forward + strafe - rotate) / denominator;

        leftFrontDrive.setPower(leftFrontPower);
        rightFrontDrive.setPower(rightFrontPower);
        leftBackDrive.setPower(leftBackPower);
        rightBackDrive.setPower(rightBackPower);
    }

    // =========================================================
    // DYNAMIC TARGET OFFSET
    // =========================================================
    double getTargetOffset(boolean isLeftShooter) {
        double distanceCm = (limelight != null && limelight.hasTarget())
                ? limelight.getDistanceFromArea()
                : 0;

        double offset = BASE_OFFSET + (distanceCm * DISTANCE_FACTOR);
        offset += isLeftShooter ? -SHOOTER_OFFSET : SHOOTER_OFFSET;
        return offset;
    }

    // =========================================================
    // ALIGNMENT CHECK
    // =========================================================
    public boolean isAligned(boolean isLeftShooter) {
        if (limelight == null || !limelight.hasTarget()) return false;
        double targetOffset = getTargetOffset(isLeftShooter);
        return Math.abs(limelight.getTx() - targetOffset) < ALIGN_DEADBAND;
    }

    // =========================================================
    // LEFT LAUNCH STATE MACHINE
    // IDLE → (ALIGN if trigger held, else) SPIN_UP → LAUNCH → LAUNCHING → IDLE
    // =========================================================
    void launchLeft(boolean shotRequested) {
        switch (leftLaunchState) {

            case IDLE:
                leftStopper.setPosition(0.7);
                if (shotRequested && limelight.hasTarget()) {
                    previousLeftAlignError = 0;
                    leftLaunchState = (gamepad1.right_trigger > 0.5)
                            ? LaunchState.ALIGN
                            : LaunchState.SPIN_UP;
                }
                break;

            case ALIGN:
                if (!limelight.hasTarget()) {
                    mecanumDrive(0, 0, 0);
                    leftLaunchState = LaunchState.IDLE;
                    break;
                }

                double leftTargetOffset = getTargetOffset(true);
                double leftTx           = limelight.getTx();
                double leftError        = leftTx - leftTargetOffset;
                double leftDeriv        = leftError - previousLeftAlignError;
                // FIX: removed double-negation — power sign now correctly
                // drives the robot toward the target
                double leftPower        = leftError * kP_align + leftDeriv * kD_align;

                if (Math.abs(leftError) < ALIGN_DEADBAND) leftPower = 0;
                leftPower = Math.max(-MAX_ALIGN_POWER, Math.min(MAX_ALIGN_POWER, leftPower));

                mecanumDrive(0, 0, leftPower);
                previousLeftAlignError = leftError;

                // Alignment telemetry — watch these while tuning kP/kD
                telemetry.addData("[L ALIGN] tx",     leftTx);
                telemetry.addData("[L ALIGN] target",  leftTargetOffset);
                telemetry.addData("[L ALIGN] error",   leftError);
                telemetry.addData("[L ALIGN] power",   leftPower);

                if (isAligned(true)) {
                    mecanumDrive(0, 0, 0);
                    previousLeftAlignError = 0;
                    leftLaunchState = LaunchState.SPIN_UP;
                }
                break;

            case SPIN_UP:
                leftLauncher.setVelocity(launcherTarget);
                rightLauncher.setVelocity(launcherTarget);

                if (leftLauncher.getVelocity() > launcherMin
                        || leftFeederTimer.seconds() > 1.0) {
                    leftLaunchState = LaunchState.LAUNCH;
                }
                break;

            case LAUNCH:
                leftStopper.setPosition(0.8);
                leftFeeder.setPower(FULL_SPEED);
                leftFeederTimer.reset();
                leftLaunchState = LaunchState.LAUNCHING;
                break;

            case LAUNCHING:
                if (leftFeederTimer.seconds() > FEED_TIME_SECONDS) {
                    leftFeeder.setPower(STOP_SPEED);
                    leftLaunchState = LaunchState.IDLE;
                }
                break;
        }
    }

    // =========================================================
    // RIGHT LAUNCH STATE MACHINE
    // IDLE → (ALIGN if trigger held, else) SPIN_UP → LAUNCH → LAUNCHING → IDLE
    // =========================================================
    void launchRight(boolean shotRequested) {
        switch (rightLaunchState) {

            case IDLE:
                rightStopper.setPosition(0.7);
                if (shotRequested && limelight.hasTarget()) {
                    previousRightAlignError = 0;
                    rightLaunchState = (gamepad1.right_trigger > 0.5)
                            ? LaunchState.ALIGN
                            : LaunchState.SPIN_UP;
                }
                break;

            case ALIGN:
                if (!limelight.hasTarget()) {
                    mecanumDrive(0, 0, 0);
                    rightLaunchState = LaunchState.IDLE;
                    break;
                }

                double rightTargetOffset = getTargetOffset(false);
                double rightTx           = limelight.getTx();
                double rightError        = rightTx - rightTargetOffset;
                double rightDeriv        = rightError - previousRightAlignError;
                // FIX: removed double-negation
                double rightPower        = rightError * kP_align + rightDeriv * kD_align;

                if (Math.abs(rightError) < ALIGN_DEADBAND) rightPower = 0;
                rightPower = Math.max(-MAX_ALIGN_POWER, Math.min(MAX_ALIGN_POWER, rightPower));

                mecanumDrive(0, 0, rightPower);
                previousRightAlignError = rightError;

                telemetry.addData("[R ALIGN] tx",     rightTx);
                telemetry.addData("[R ALIGN] target",  rightTargetOffset);
                telemetry.addData("[R ALIGN] error",   rightError);
                telemetry.addData("[R ALIGN] power",   rightPower);

                if (isAligned(false)) {
                    mecanumDrive(0, 0, 0);
                    previousRightAlignError = 0;
                    rightLaunchState = LaunchState.SPIN_UP;
                }
                break;

            case SPIN_UP:
                leftLauncher.setVelocity(launcherTarget);
                rightLauncher.setVelocity(launcherTarget);

                if (rightLauncher.getVelocity() > launcherMin) {
                    rightLaunchState = LaunchState.LAUNCH;
                }
                break;

            case LAUNCH:
                rightStopper.setPosition(0.2);
                rightFeeder.setPower(FULL_SPEED);
                rightFeederTimer.reset();
                rightLaunchState = LaunchState.LAUNCHING;
                break;

            case LAUNCHING:
                if (rightFeederTimer.seconds() > FEED_TIME_SECONDS) {
                    rightFeeder.setPower(STOP_SPEED);
                    rightLaunchState = LaunchState.IDLE;
                }
                break;
        }
    }

    // =========================================================
    // MANUAL LEFT LAUNCH STATE MACHINE (no alignment)
    // GP2 RIGHT BUMPER triggers this.
    // =========================================================
    void manualLaunchLeft(boolean shotRequested) {
        switch (manualLeftState) {

            case IDLE:
                if (shotRequested) {
                    if (limelight.hasTarget()) {
                        double distanceCm = limelight.getDistanceFromArea();
                        double rpm        = ShooterModel.distanceToRPM(distanceCm, true);
                        manualLeftTarget  = rpm * 28.0 / 60.0;
                    } else {
                        manualLeftTarget = launcherTarget;
                    }
                    manualLeftState = ManualLaunchState.SPIN_UP;
                }
                break;

            case SPIN_UP:
                leftLauncher.setVelocity(manualLeftTarget);
                rightLauncher.setVelocity(manualLeftTarget);

                if (leftLauncher.getVelocity() > manualLeftTarget * 0.95) {
                    manualLeftState = ManualLaunchState.LAUNCH;
                }
                break;

            case LAUNCH:
                leftFeeder.setPower(FULL_SPEED);
                leftFeederTimer.reset();
                manualLeftState = ManualLaunchState.LAUNCHING;
                break;

            case LAUNCHING:
                if (leftFeederTimer.seconds() > FEED_TIME_SECONDS) {
                    leftFeeder.setPower(STOP_SPEED);
                    leftLauncher.setVelocity(0);
                    rightLauncher.setVelocity(0);
                    manualLeftState = ManualLaunchState.IDLE;
                }
                break;
        }
    }

    // =========================================================
    // MANUAL RIGHT LAUNCH STATE MACHINE (no alignment)
    // GP2 LEFT BUMPER triggers this.
    // =========================================================
    void manualLaunchRight(boolean shotRequested) {
        switch (manualRightState) {

            case IDLE:
                if (shotRequested) {
                    if (limelight.hasTarget()) {
                        double distanceCm = limelight.getDistanceFromArea();
                        double rpm        = ShooterModel.distanceToRPM(distanceCm, false);
                        manualRightTarget = rpm * 28.0 / 60.0;
                    } else {
                        manualRightTarget = launcherTarget;
                    }
                    manualRightState = ManualLaunchState.SPIN_UP;
                }
                break;

            case SPIN_UP:
                leftLauncher.setVelocity(manualRightTarget);
                rightLauncher.setVelocity(manualRightTarget);

                if (rightLauncher.getVelocity() > manualRightTarget * 0.95) {
                    manualRightState = ManualLaunchState.LAUNCH;
                }
                break;

            case LAUNCH:
                rightFeeder.setPower(FULL_SPEED);
                rightFeederTimer.reset();
                manualRightState = ManualLaunchState.LAUNCHING;
                break;

            case LAUNCHING:
                if (rightFeederTimer.seconds() > FEED_TIME_SECONDS) {
                    rightFeeder.setPower(STOP_SPEED);
                    leftLauncher.setVelocity(0);
                    rightLauncher.setVelocity(0);
                    manualRightState = ManualLaunchState.IDLE;
                }
                break;
        }
    }

    @Override
    public void stop() {
        leftLauncher.setVelocity(0);
        rightLauncher.setVelocity(0);
        leftFeeder.setPower(STOP_SPEED);
        rightFeeder.setPower(STOP_SPEED);
        intake1.setVelocity(0);
    }
}