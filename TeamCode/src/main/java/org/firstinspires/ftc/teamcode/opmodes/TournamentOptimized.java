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
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.teamcode.vision.LimelightVision;
import org.firstinspires.ftc.teamcode.math.ShooterModel;

/**
 * Tournament Optimized Shooting OpMode.
 * Features:
 * - Single Driver Mecanum Drivetrain
 * - Limelight distance-based RPM calculation
 * - D-Pad RPM tuning (±300 RPM deviation bar)
 * - Simplified hardware (One shooter, intake, feeder)
 */
@TeleOp(name = "Tournament Optimized", group = "Production")
public class TournamentOptimized extends OpMode {

    // --- HARDWARE ---
    private DcMotor leftFrontDrive, rightFrontDrive, leftBackDrive, rightBackDrive;
    private DcMotorEx leftLauncher, rightLauncher, intake;
    private CRServo leftFeeder, rightFeeder;
    private Servo rgbIndicator;
    private LimelightVision limelight;

    // --- CONFIGURATION & CONSTANTS ---
    private static final double TICKS_PER_REV = 28.0;
    private static final double SECONDS_PER_MINUTE = 60.0;
    
    private static final double FEED_TIME_SECONDS = 1.0;
    private static final double FULL_SPEED = 1.0;
    private static final double STOP_SPEED = 0.0;

    private static final double RPM_DEVIATION_RANGE = 1000.0; // ±1000 RPM
    private static final double TUNING_STEP = 0.05; // 5% per press

    // RGB Constants
    private static final double RGB_ALIGNMENT_THRESHOLD = 10.0; // degrees
    private static final double RGB_GREEN_POSITION = 0.5;
    private static final double RGB_RED_POSITION = 0.28;
    private static final double ALIGN_DEADBAND = 0.5;

    // --- STATE VARIABLES ---
    private double tuningPercent = 0.5; // 50% = No deviation
    private double baseLimelightRPM = 3000.0;
    private double targetRPM = 3000.0;

    private enum SelectedShooter { LEFT, RIGHT }
    private SelectedShooter activeShooter = SelectedShooter.RIGHT; // Defaulting to one side

    private enum LaunchState { IDLE, SPIN_UP, LAUNCH, LAUNCHING }
    private LaunchState launchState = LaunchState.IDLE;
    private ElapsedTime launchTimer = new ElapsedTime();

    private boolean intakeOn = false;
    private boolean lastDpadRight = false;
    private boolean lastDpadLeft = false;
    private boolean lastDpadUp = false;
    private boolean lastA = false;

    @Override
    public void init() {
        // Drivetrain
        leftFrontDrive  = hardwareMap.get(DcMotor.class, "left_front_drive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
        leftBackDrive   = hardwareMap.get(DcMotor.class, "left_back_drive");
        rightBackDrive  = hardwareMap.get(DcMotor.class, "right_back_drive");

        // Launcher & Intake
        leftLauncher  = hardwareMap.get(DcMotorEx.class, "left_launcher");
        rightLauncher = hardwareMap.get(DcMotorEx.class, "right_launcher");
        intake        = hardwareMap.get(DcMotorEx.class, "intake1");

        // Feeders & Indicator
        leftFeeder   = hardwareMap.get(CRServo.class, "left_feeder");
        rightFeeder  = hardwareMap.get(CRServo.class, "right_feeder");
        rgbIndicator = hardwareMap.get(Servo.class, "rgb_indicator");

        limelight = new LimelightVision(hardwareMap);

        // Directions
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        leftLauncher.setDirection(DcMotorSimple.Direction.FORWARD);
        rightLauncher.setDirection(DcMotorSimple.Direction.FORWARD);
        intake.setDirection(DcMotorSimple.Direction.FORWARD);

        // Motor Modes & PID
        leftLauncher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightLauncher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftLauncher.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(300, 0, 0, 10));
        rightLauncher.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(300, 0, 0, 10));

        leftFrontDrive.setZeroPowerBehavior(BRAKE);
        rightFrontDrive.setZeroPowerBehavior(BRAKE);
        leftBackDrive.setZeroPowerBehavior(BRAKE);
        rightBackDrive.setZeroPowerBehavior(BRAKE);
        leftLauncher.setZeroPowerBehavior(BRAKE);
        rightLauncher.setZeroPowerBehavior(BRAKE);

        telemetry.addData("Status", "Tournament Optimized Initialized");
    }

    @Override
    public void loop() {
        limelight.update();

        handleDrivetrain();
        handleLimelightRPM();
        handleTuning();
        handleIntake();
        handleLaunchSequence();
        updateRGBIndicator();
        updateTelemetry();
    }

    private void handleDrivetrain() {
        double forward = -gamepad1.left_stick_y;
        double strafe  = gamepad1.left_stick_x;
        double rotate  = gamepad1.right_stick_x;

        double denominator = Math.max(Math.abs(forward) + Math.abs(strafe) + Math.abs(rotate), 1);
        leftFrontDrive.setPower((forward + strafe + rotate) / denominator);
        rightFrontDrive.setPower((forward - strafe - rotate) / denominator);
        leftBackDrive.setPower((forward - strafe + rotate) / denominator);
        rightBackDrive.setPower((forward + strafe - rotate) / denominator);
    }

    private void handleLimelightRPM() {
        if (limelight.hasTarget()) {
            double distanceCm = limelight.getDistanceFromArea();
            // Calculate base RPM based on distance model (using true for left/main model)
            baseLimelightRPM = ShooterModel.distanceToRPM(distanceCm, true);
        } else {
            // Default idle RPM if no target found
            baseLimelightRPM = 3000.0;
        }
        
        // Final target includes the manual deviation (±300 RPM)
        // 50% tuning = 0 deviation. 100% = +300. 0% = -300.
        double deviation = (tuningPercent - 0.5) * 2.0 * RPM_DEVIATION_RANGE;
        targetRPM = baseLimelightRPM + deviation;
    }

    private void handleTuning() {
        // D-Pad Right to increase tuning bar
        if (gamepad1.dpad_right && !lastDpadRight) {
            tuningPercent = Range.clip(tuningPercent + TUNING_STEP, 0.0, 1.0);
        }
        lastDpadRight = gamepad1.dpad_right;

        // D-Pad Left to decrease tuning bar
        if (gamepad1.dpad_left && !lastDpadLeft) {
            tuningPercent = Range.clip(tuningPercent - TUNING_STEP, 0.0, 1.0);
        }
        lastDpadLeft = gamepad1.dpad_left;

        // D-Pad Up to toggle which shooter is active
        if (gamepad1.dpad_up && !lastDpadUp) {
            activeShooter = (activeShooter == SelectedShooter.LEFT) ? SelectedShooter.RIGHT : SelectedShooter.LEFT;
        }
        lastDpadUp = gamepad1.dpad_up;
    }

    private void handleIntake() {
        if (gamepad1.a && !lastA) {
            intakeOn = !intakeOn;
            intake.setVelocity(intakeOn ? 6000 : 0);
        }
        lastA = gamepad1.a;
        
        // Manual override for reverse intake (X button)
        if (gamepad1.x) {
            intake.setVelocity(-6000);
            intakeOn = false;
        } else if (!intakeOn) {
            intake.setVelocity(0);
        }
    }

    private void handleLaunchSequence() {
        double velocityTicks = (targetRPM * TICKS_PER_REV) / SECONDS_PER_MINUTE;

        switch (launchState) {
            case IDLE:
                if (gamepad1.right_bumper) {
                    launchState = LaunchState.SPIN_UP;
                } else {
                    leftLauncher.setVelocity(0);
                    rightLauncher.setVelocity(0);
                    leftFeeder.setPower(STOP_SPEED);
                    rightFeeder.setPower(STOP_SPEED);
                }
                break;

            case SPIN_UP:
                // Always spin both launchers for every shot
                leftLauncher.setVelocity(velocityTicks);
                rightLauncher.setVelocity(velocityTicks);

                // Check if both are up to speed (95% threshold)
                if (leftLauncher.getVelocity() > velocityTicks * 0.95 && 
                    rightLauncher.getVelocity() > velocityTicks * 0.95) {
                    launchState = LaunchState.LAUNCH;
                }
                
                // Cancel if bumper released
                if (!gamepad1.right_bumper) launchState = LaunchState.IDLE;
                break;

            case LAUNCH:
                if (activeShooter == SelectedShooter.LEFT) {
                    leftFeeder.setPower(FULL_SPEED);
                } else {
                    rightFeeder.setPower(FULL_SPEED);
                }
                launchTimer.reset();
                launchState = LaunchState.LAUNCHING;
                break;

            case LAUNCHING:
                if (launchTimer.seconds() > FEED_TIME_SECONDS) {
                    leftFeeder.setPower(STOP_SPEED);
                    rightFeeder.setPower(STOP_SPEED);
                    launchState = LaunchState.IDLE;
                }
                break;
        }
    }

    private void updateRGBIndicator() {
        if (!limelight.hasTarget()) {
            rgbIndicator.setPosition(RGB_RED_POSITION);
            return;
        }

        // Calculate alignment error based on active shooter
        // (Assuming 0 is the center target for now)
        double alignmentError = Math.abs(limelight.getTx());

        double servoPosition;
        if (alignmentError <= ALIGN_DEADBAND) {
            servoPosition = RGB_GREEN_POSITION;
        } else if (alignmentError >= RGB_ALIGNMENT_THRESHOLD) {
            servoPosition = RGB_RED_POSITION;
        } else {
            double errorRatio = alignmentError / RGB_ALIGNMENT_THRESHOLD;
            servoPosition = RGB_GREEN_POSITION - (errorRatio * (RGB_GREEN_POSITION - RGB_RED_POSITION));
        }
        rgbIndicator.setPosition(servoPosition);
    }

    private void updateTelemetry() {
        telemetry.addLine("=== TOURNAMENT OPTIMIZED STATUS ===");
        telemetry.addData("Active Shooter", activeShooter);
        telemetry.addData("Limelight Target", limelight.hasTarget() ? "LOCKED" : "SEARCHING...");
        if (limelight.hasTarget()) {
            telemetry.addData("Distance (cm)", "%.1f", limelight.getDistanceFromArea());
            telemetry.addData("Horizontal Error (tx)", "%.1f", limelight.getTx());
        }
        
        telemetry.addLine("\n=== RPM TUNING (±1000 RPM) ===");
        String bar = "[";
        int barPos = (int)(tuningPercent * 20);
        for (int i = 0; i < 20; i++) {
            if (i == barPos) bar += "█";
            else if (i == 10) bar += "|";
            else bar += "-";
        }
        bar += "]";
        telemetry.addLine(bar);
        telemetry.addData("Manual Adjustment", "%.0f%%", tuningPercent * 100);
        telemetry.addData("Limelight Base", "%.0f RPM", baseLimelightRPM);
        telemetry.addData("Final Target", "%.0f RPM", targetRPM);

        telemetry.addLine("\n=== MOTOR PERFORMANCE ===");
        double leftRPM = Math.abs(leftLauncher.getVelocity() * 60 / 28);
        double rightRPM = Math.abs(rightLauncher.getVelocity() * 60 / 28);
        telemetry.addData("Left RPM", "%.0f", leftRPM);
        telemetry.addData("Right RPM", "%.0f", rightRPM);
        telemetry.addData("Intake", intakeOn ? "ON" : "OFF");

        telemetry.update();
    }
}
