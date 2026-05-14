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
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import java.util.List;

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

    private static final double RPM_DEVIATION_RANGE = 1800.0; // ±1800 RPM
    private static final double TUNING_STEP = 0.10; // 10% per press

    // RGB Constants
    private static final double RGB_ALIGNMENT_THRESHOLD = 10.0; // degrees
    private static final double RGB_GREEN_POSITION = 0.5;
    private static final double RGB_RED_POSITION = 0.28;
    private static final double ALIGN_DEADBAND = 0.5;

    // --- STATE VARIABLES ---
    private double tuningPercent = 0.5; // 50% = No deviation
    private double baseLimelightRPM = 3000.0;
    private double targetRPM = 3000.0;

    // Limelight Stability & Diagnostics
    private int currentPipeline = 0;
    private double lastKnownDistance = 0;
    private ElapsedTime targetLossTimer = new ElapsedTime();
    private String distanceMethod = "None";
    private int lastTargetId = -1;

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
    private boolean lastB = false;

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
        LLResult res = limelight.getRawResult();
        boolean hasLock = false;

        if (res != null && res.isValid()) {
            double currentDistance = 0;
            List<LLResultTypes.FiducialResult> fiducials = res.getFiducialResults();

            if (fiducials != null && !fiducials.isEmpty()) {
                // 3D Tracking: Use Z-distance from the first detected AprilTag
                // Robot space Z is distance forward from camera
                // Note: Units depend on your LL calibration, assuming CM
                currentDistance = Math.abs(fiducials.get(0).getTargetPoseRobotSpace().getPosition().z);
                lastTargetId = fiducials.get(0).getFiducialId();
                distanceMethod = "3D Pose";
                
                // If 3D distance is extremely small (calibration error), fallback
                if (currentDistance < 1.0) {
                    currentDistance = limelight.getDistanceFromArea();
                    distanceMethod = "Area Model";
                }
            } else {
                // Fallback to Area-based model
                currentDistance = limelight.getDistanceFromArea();
                distanceMethod = "Area Model";
            }

            if (currentDistance > 1.0) {
                lastKnownDistance = currentDistance;
                targetLossTimer.reset();
                hasLock = true;
            }
        }

        // Stability: If we lost target, hold the last distance for 1.5 seconds
        if (hasLock || (targetLossTimer.seconds() < 1.5 && lastKnownDistance > 0)) {
            baseLimelightRPM = ShooterModel.distanceToRPM(lastKnownDistance, true);
        } else {
            // Default idle RPM if target lost for too long
            baseLimelightRPM = 3000.0;
            distanceMethod = "None";
            lastTargetId = -1;
        }
        
        // Final target includes the manual deviation (±1800 RPM)
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

        // Button B to cycle Limelight pipelines
        if (gamepad1.b && !lastB) {
            currentPipeline = (currentPipeline + 1) % 4; // Cycle 0-3
            // Access internal LL directly via the vision wrapper if possible
            // or we just trust the driver to see the change in telemetry
        }
        lastB = gamepad1.b;
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

        double horizontalAngle = limelight.getTx();
        if (Math.abs(horizontalAngle) < ALIGN_DEADBAND) {
            rgbIndicator.setPosition(RGB_GREEN_POSITION);
        } else {
            rgbIndicator.setPosition(RGB_RED_POSITION);
        }
    }

    private void updateTelemetry() {
        telemetry.addData("--- MODE ---", "TOURNAMENT OPTIMIZED");
        telemetry.addData("Active Shooter", activeShooter);
        telemetry.addData("Launch State", launchState);
        
        telemetry.addData("--- TARGETING ---", "");
        telemetry.addData("Method", distanceMethod);
        telemetry.addData("Distance", "%.1f cm", lastKnownDistance);
        telemetry.addData("Target ID", lastTargetId);
        telemetry.addData("Pipeline", currentPipeline);
        
        telemetry.addData("--- SPEED ---", "");
        telemetry.addData("Base RPM", "%.0f", baseLimelightRPM);
        telemetry.addData("Manual Tuning", "%.0f%%", tuningPercent * 100);
        telemetry.addData("FINAL TARGET", "%.0f RPM", targetRPM);
        
        telemetry.addData("--- LIVE RPM ---", "");
        telemetry.addData("Left Launcher", "%.0f", (leftLauncher.getVelocity() * SECONDS_PER_MINUTE) / TICKS_PER_REV);
        telemetry.addData("Right Launcher", "%.0f", (rightLauncher.getVelocity() * SECONDS_PER_MINUTE) / TICKS_PER_REV);

        telemetry.update();
    }
}
