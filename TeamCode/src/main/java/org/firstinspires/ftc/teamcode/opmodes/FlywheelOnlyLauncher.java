package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

/**
 * Simplified TeleOp that only controls the two flywheel motors.
 * Derived from MegaHawkLauncher.
 */
@TeleOp(name = "Flywheel Only Launcher", group = "Production")
public class FlywheelOnlyLauncher extends OpMode {

    // --- HARDWARE ---
    private DcMotorEx topFlywheel;
    private DcMotorEx bottomFlywheel;

    // --- CONSTANTS ---
    private static final double TICKS_PER_REV = 28.0;
    private static final double SECONDS_PER_MINUTE = 60.0;

    private static final double RPM_OFFSET = 3000.0;
    private static final double MIN_RPM = 0.0;
    private static final double MAX_RPM = 6000.0;

    private static final double STEP_NORMAL = 100.0;
    private static final double STEP_PRECISION = 10.0;

    private static final double RPM_TOLERANCE = 100.0;

    // --- STATE VARIABLES ---
    private double targetBottomRPM = 6000.0;
    private double targetTopRPM = 3000.0;

    private enum SelectedFlywheel { BOTTOM, TOP }
    private SelectedFlywheel currentSelection = SelectedFlywheel.BOTTOM;

    private boolean lastDpadUp = false;
    private boolean lastDpadRight = false;
    private boolean lastDpadLeft = false;

    @Override
    public void init() {
        // Flywheel Mapping
        topFlywheel = hardwareMap.get(DcMotorEx.class, "topFlywheel");
        bottomFlywheel = hardwareMap.get(DcMotorEx.class, "bottomFlywheel");

        // Motor Configuration
        topFlywheel.setDirection(DcMotorEx.Direction.FORWARD);
        bottomFlywheel.setDirection(DcMotorEx.Direction.FORWARD);

        topFlywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bottomFlywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        telemetry.addData("Status", "Hardware Initialized - Flywheels Only");
    }

    @Override
    public void loop() {
        handleInputs();
        updateMotorVelocity();
        updateTelemetry();
    }

    private void handleInputs() {
        double currentStep = gamepad1.a ? STEP_PRECISION : STEP_NORMAL;

        // EMERGENCY STOP: Set both targets to 0
        if (gamepad1.b) {
            targetBottomRPM = 0;
            targetTopRPM = 0;
        }

        // Switch selection between top and bottom flywheel
        if (gamepad1.dpad_up && !lastDpadUp) {
            currentSelection = (currentSelection == SelectedFlywheel.BOTTOM) ? SelectedFlywheel.TOP : SelectedFlywheel.BOTTOM;
        }
        lastDpadUp = gamepad1.dpad_up;

        // Increase RPM
        if (gamepad1.dpad_right && !lastDpadRight) {
            adjustRPM(currentStep);
        }
        lastDpadRight = gamepad1.dpad_right;

        // Decrease RPM
        if (gamepad1.dpad_left && !lastDpadLeft) {
            adjustRPM(-currentStep);
        }
        lastDpadLeft = gamepad1.dpad_left;
    }

    private void adjustRPM(double step) {
        // If restarting from a stop (0 RPM), jump to minimum operational levels
        if (targetBottomRPM == 0 && step > 0) {
            targetBottomRPM = RPM_OFFSET;
            targetTopRPM = 0;
            return;
        }

        if (currentSelection == SelectedFlywheel.BOTTOM) {
            targetBottomRPM += step;
            targetBottomRPM = Math.max(RPM_OFFSET, Math.min(MAX_RPM, targetBottomRPM));
            targetTopRPM = targetBottomRPM - RPM_OFFSET;
        } else {
            targetTopRPM += step;
            targetTopRPM = Math.max(MIN_RPM, Math.min(MAX_RPM - RPM_OFFSET, targetTopRPM));
            targetBottomRPM = targetTopRPM + RPM_OFFSET;
        }
    }

    private void updateMotorVelocity() {
        topFlywheel.setVelocity(rpmToTicksPerSecond(targetTopRPM));
        bottomFlywheel.setVelocity(rpmToTicksPerSecond(targetBottomRPM));
    }

    private double rpmToTicksPerSecond(double rpm) {
        return (rpm * TICKS_PER_REV) / SECONDS_PER_MINUTE;
    }

    private double ticksPerSecondToRPM(double tps) {
        return (tps * SECONDS_PER_MINUTE) / TICKS_PER_REV;
    }

    private void updateTelemetry() {
        double actualBottomRPM = ticksPerSecondToRPM(bottomFlywheel.getVelocity());
        double actualTopRPM = ticksPerSecondToRPM(topFlywheel.getVelocity());

        double bottomError = targetBottomRPM - actualBottomRPM;
        double topError = targetTopRPM - actualTopRPM;

        boolean bottomReady = targetBottomRPM > 0 && Math.abs(bottomError) < RPM_TOLERANCE;
        boolean topReady = targetTopRPM > 0 && Math.abs(topError) < RPM_TOLERANCE;

        telemetry.addLine("=== FLYWHEEL PERFORMANCE ===");
        if (targetBottomRPM == 0 && targetTopRPM == 0) {
            telemetry.addData("STATUS", "STOPPED");
        } else {
            telemetry.addData("STATUS", (bottomReady && topReady) ? "READY" : "SPINNING UP...");
        }
        telemetry.addData("Bottom (Target|Actual)", "%.0f | %.0f RPM", targetBottomRPM, actualBottomRPM);
        telemetry.addData("Top    (Target|Actual)", "%.0f | %.0f RPM", targetTopRPM, actualTopRPM);
        
        telemetry.addLine("\n=== CONTROL INTERFACE ===");
        telemetry.addData("Stop Motors", "(B)");
        telemetry.addData("Selected Flywheel", currentSelection);
        telemetry.addData("Precision Mode (A)", gamepad1.a ? "ACTIVE (10 RPM)" : "INACTIVE (100 RPM)");
        
        telemetry.update();
    }
}
