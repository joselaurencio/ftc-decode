package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

/**
 * Simplified TeleOp that only controls the two flywheel motors.
 * Configuration (RPM) happens during INIT LOOP.
 * Both flywheels run at the same speed.
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

    private static final double RPM_TOLERANCE = 100.0;

    // --- STATE VARIABLES ---
    private double targetRPM = 3000.0;
    private boolean motorsStopped = false;

    private boolean lastDpadRight = false;
    private boolean lastDpadLeft = false;

    @Override
    public void init() {
        // Flywheel Mapping
        topFlywheel = hardwareMap.get(DcMotorEx.class, "topFlywheel");
        bottomFlywheel = hardwareMap.get(DcMotorEx.class, "bottomFlywheel");

        // Reset Encoders to ensure clean start for PID
        topFlywheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bottomFlywheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        
        // Re-enable encoder mode
        topFlywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bottomFlywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Motor Configuration: 
        // We're setting Top to REVERSE and Bottom to FORWARD based on runaway feedback.
        topFlywheel.setDirection(DcMotorEx.Direction.REVERSE);
        bottomFlywheel.setDirection(DcMotorEx.Direction.FORWARD);

        // Set BRAKE behavior
        topFlywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bottomFlywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addData("Status", "Hardware Initialized - Encoders Reset");
        telemetry.update();
    }

    @Override
    public void init_loop() {
        handleRPMAdjustments();
        
        telemetry.addLine("=== PRE-START CONFIGURATION ===");
        telemetry.addData("Target RPM", "%.0f", targetRPM);
        telemetry.addLine("\nUse DPAD LEFT/RIGHT to adjust");
        telemetry.addData("Precision Mode (A)", gamepad1.a ? "ACTIVE (10 RPM)" : "INACTIVE (100 RPM)");
        telemetry.addLine("\nPress PLAY to start motors at this speed");
        telemetry.update();
    }

    @Override
    public void start() {
        motorsStopped = false;
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
        // Allow adjustments even during runtime
        handleRPMAdjustments();

        // EMERGENCY STOP: Stop motors instantly
        if (gamepad1.b) {
            motorsStopped = true;
        }
        
        // Restart motors if they were stopped
        if (gamepad1.x) {
            motorsStopped = false;
        }
    }

    private void updateMotorVelocity() {
        if (motorsStopped) {
            topFlywheel.setPower(0);
            bottomFlywheel.setPower(0);
        } else {
            double velocityTicks = (targetRPM * TICKS_PER_REV) / SECONDS_PER_MINUTE;
            topFlywheel.setVelocity(velocityTicks);
            bottomFlywheel.setVelocity(velocityTicks);
        }
    }

    private double ticksPerSecondToRPM(double tps) {
        return (tps * SECONDS_PER_MINUTE) / TICKS_PER_REV;
    }

    private void updateTelemetry() {
        // Using absolute values for display
        double actualBottomRPM = Math.abs(ticksPerSecondToRPM(bottomFlywheel.getVelocity()));
        double actualTopRPM = Math.abs(ticksPerSecondToRPM(topFlywheel.getVelocity()));

        double bottomError = targetRPM - actualBottomRPM;
        double topError = targetRPM - actualTopRPM;

        boolean ready = !motorsStopped && 
                        Math.abs(bottomError) < RPM_TOLERANCE && 
                        Math.abs(topError) < RPM_TOLERANCE;

        telemetry.addLine("=== FLYWHEEL PERFORMANCE ===");
        if (motorsStopped) {
            telemetry.addData("STATUS", "STOPPED (Press X to Resume)");
        } else {
            telemetry.addData("STATUS", ready ? "READY" : "SPINNING UP...");
        }
        telemetry.addData("Target RPM", "%.0f", targetRPM);
        telemetry.addData("Bottom Actual", "%.0f RPM (Vel: %.1f)", actualBottomRPM, bottomFlywheel.getVelocity());
        telemetry.addData("Top Actual", "%.0f RPM (Vel: %.1f)", actualTopRPM, topFlywheel.getVelocity());
        
        telemetry.addLine("\n=== CONTROLS ===");
        telemetry.addData("Adjust RPM", "DPAD L/R");
        telemetry.addData("Stop Motors", "B");
        telemetry.addData("Resume Motors", "X");
        
        telemetry.update();
    }
}
