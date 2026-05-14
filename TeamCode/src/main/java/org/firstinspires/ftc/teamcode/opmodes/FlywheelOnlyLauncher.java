package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

/**
 * Simplified TeleOp that only controls the two flywheel motors.
 * Uses a software sync loop to ensure both motors reach the exact same RPM.
 */
@TeleOp(name = "Flywheel Only Launcher", group = "Production")
public class FlywheelOnlyLauncher extends OpMode {

    // --- HARDWARE ---
    private DcMotorEx topFlywheel;
    private DcMotorEx bottomFlywheel;

    // --- CONSTANTS ---
    private static final double TICKS_PER_REV = 28.0;
    private static final double SECONDS_PER_MINUTE = 60.0;

    private static final double MIN_RPM = 0.0;
    private static final double MAX_RPM = 6000.0;

    private static final double STEP_NORMAL = 100.0;
    private static final double STEP_PRECISION = 10.0;

    private static final double RPM_TOLERANCE = 50.0;
    
    // Tuning constant for software synchronization
    private static final double SYNC_GAIN = 0.00005; 

    // --- STATE VARIABLES ---
    private double targetRPM = 3000.0;
    private boolean motorsStopped = false;
    private double topManualPower = 0.0;

    private boolean lastDpadRight = false;
    private boolean lastDpadLeft = false;

    @Override
    public void init() {
        // Flywheel Mapping
        topFlywheel = hardwareMap.get(DcMotorEx.class, "topFlywheel");
        bottomFlywheel = hardwareMap.get(DcMotorEx.class, "bottomFlywheel");

        // Reset Encoders
        topFlywheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bottomFlywheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        
        // MODE CONFIGURATION:
        // Top uses RUN_WITHOUT_ENCODER to avoid hardware runaway, sync handled in software loop
        topFlywheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        // Bottom uses hardware velocity control for precision
        bottomFlywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Opposite Directions for shooter setup
        topFlywheel.setDirection(DcMotorEx.Direction.REVERSE);
        bottomFlywheel.setDirection(DcMotorEx.Direction.FORWARD);

        // Set BRAKE behavior
        topFlywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bottomFlywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addData("Status", "Hardware Initialized - Exact Sync Enabled");
        telemetry.update();
    }

    @Override
    public void init_loop() {
        handleRPMAdjustments();
        
        telemetry.addLine("=== PRE-START CONFIGURATION ===");
        telemetry.addData("Target RPM", "%.0f", targetRPM);
        telemetry.addLine("\nUse DPAD LEFT/RIGHT to adjust");
        telemetry.addData("Precision Mode (A)", gamepad1.a ? "ACTIVE (10 RPM)" : "INACTIVE (100 RPM)");
        telemetry.addLine("\nPress PLAY to start motors");
        telemetry.update();
    }

    @Override
    public void start() {
        motorsStopped = false;
        topManualPower = targetRPM / MAX_RPM; // Initial guess
    }

    @Override
    public void loop() {
        handleRuntimeInputs();
        updateMotorVelocity();
        updateTelemetry();
    }

    private void handleRPMAdjustments() {
        double currentStep = gamepad1.a ? STEP_PRECISION : STEP_NORMAL;

        // Increase RPM
        if (gamepad1.dpad_right && !lastDpadRight) {
            targetRPM += currentStep;
        }
        lastDpadRight = gamepad1.dpad_right;

        // Decrease RPM
        if (gamepad1.dpad_left && !lastDpadLeft) {
            targetRPM -= currentStep;
        }
        lastDpadLeft = gamepad1.dpad_left;

        // Clamp RPM
        targetRPM = Math.max(MIN_RPM, Math.min(MAX_RPM, targetRPM));
    }

    private void handleRuntimeInputs() {
        handleRPMAdjustments();

        // EMERGENCY STOP
        if (gamepad1.b) {
            motorsStopped = true;
            topManualPower = 0;
        }
        
        // RESUME
        if (gamepad1.x) {
            motorsStopped = false;
            topManualPower = targetRPM / MAX_RPM;
        }
    }

    private void updateMotorVelocity() {
        if (motorsStopped) {
            topFlywheel.setPower(0);
            bottomFlywheel.setPower(0);
        } else {
            // BOTTOM MOTOR: Velocity Control
            double velocityTicks = (targetRPM * TICKS_PER_REV) / SECONDS_PER_MINUTE;
            bottomFlywheel.setVelocity(velocityTicks);
            
            // TOP MOTOR: Software Synchronization Loop
            double actualTopRPM = Math.abs(ticksPerSecondToRPM(topFlywheel.getVelocity()));
            double error = targetRPM - actualTopRPM;
            
            // Adjust power slightly each loop until RPM matches target
            topManualPower += error * SYNC_GAIN;
            
            // Safety Clamp
            topManualPower = Math.max(0, Math.min(1.0, topManualPower));
            topFlywheel.setPower(topManualPower);
        }
    }

    private double ticksPerSecondToRPM(double tps) {
        return (tps * SECONDS_PER_MINUTE) / TICKS_PER_REV;
    }

    private void updateTelemetry() {
        double actualBottomRPM = Math.abs(ticksPerSecondToRPM(bottomFlywheel.getVelocity()));
        double actualTopRPM = Math.abs(ticksPerSecondToRPM(topFlywheel.getVelocity()));

        double syncError = Math.abs(actualBottomRPM - actualTopRPM);

        telemetry.addLine("=== SYNC PERFORMANCE ===");
        if (motorsStopped) {
            telemetry.addData("STATUS", "STOPPED (X to Resume)");
        } else {
            telemetry.addData("STATUS", syncError < RPM_TOLERANCE ? "LOCKED" : "SYNCING...");
        }
        telemetry.addData("Target RPM", "%.0f", targetRPM);
        telemetry.addData("Bottom RPM", "%.0f", actualBottomRPM);
        telemetry.addData("Top RPM   ", "%.0f", actualTopRPM);
        telemetry.addData("Sync Error", "%.0f RPM", syncError);
        
        telemetry.addLine("\n=== DEBUG ===");
        telemetry.addData("Top Power", "%.4f", topManualPower);
        
        telemetry.update();
    }
}
