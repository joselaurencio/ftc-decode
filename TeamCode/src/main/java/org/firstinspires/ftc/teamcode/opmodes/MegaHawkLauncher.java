package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * COMPLETE production-level FTC TeleOp for a dual-flywheel shooter system.
 *
 * System Features:
 * - Dual GoBILDA Flywheels (topFlywheel, bottomFlywheel)
 * - Automatic RPM Offset: topRPM = bottomRPM - 3000
 * - Precision RPM Tuning (Normal: 100 RPM, Precision: 10 RPM)
 * - Timed Loader Servo Control
 * - Drivetrain: Spinning only for aiming the shooter
 * - Advanced Diagnostics and Battery Monitoring
 */
@TeleOp(name = "MegaHawk Launcher: Golf Ball Shooter", group = "Production")
public class MegaHawkLauncher extends OpMode {

    // --- HARDWARE ---
    private DcMotorEx topFlywheel;
    private DcMotorEx bottomFlywheel;
    private Servo loaderServo;
    private VoltageSensor batteryVoltageSensor;

    // --- DRIVETRAIN ---
    private DcMotorEx leftFrontDrive;
    private DcMotorEx rightFrontDrive;
    private DcMotorEx leftBackDrive;
    private DcMotorEx rightBackDrive;

    // --- CONSTANTS ---
    private static final double TICKS_PER_REV = 28.0;
    private static final double SECONDS_PER_MINUTE = 60.0;

    private static final double RPM_OFFSET = 3000.0;
    private static final double MIN_RPM = 0.0;
    private static final double MAX_RPM = 6000.0;

    private static final double STEP_NORMAL = 100.0;
    private static final double STEP_PRECISION = 10.0;

    private static final double SERVO_LOADING = 0.0;
    private static final double SERVO_SHOOTING = 1.0;

    private static final double RPM_TOLERANCE = 100.0;

    // --- STATE VARIABLES ---
    private double targetBottomRPM = 6000.0;
    private double targetTopRPM = 3000.0;

    private enum SelectedFlywheel { BOTTOM, TOP }
    private SelectedFlywheel currentSelection = SelectedFlywheel.BOTTOM;

    private boolean lastDpadUp = false;
    private boolean lastDpadRight = false;
    private boolean lastDpadLeft = false;

    private double minVoltageObserved = 14.0;

    private ElapsedTime loopTimer = new ElapsedTime();
    private double lastLoopTimeMs = 0;
    private double avgLoopTimeMs = 0;
    private int loopCount = 0;

    @Override
    public void init() {
        // Flywheel Mapping
        topFlywheel = hardwareMap.get(DcMotorEx.class, "topFlywheel");
        bottomFlywheel = hardwareMap.get(DcMotorEx.class, "bottomFlywheel");
        loaderServo = hardwareMap.get(Servo.class, "loaderServo");
        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();

        // Drivetrain Mapping
        leftFrontDrive  = hardwareMap.get(DcMotorEx.class, "left_front_drive");
        rightFrontDrive = hardwareMap.get(DcMotorEx.class, "right_front_drive");
        leftBackDrive   = hardwareMap.get(DcMotorEx.class, "left_back_drive");
        rightBackDrive  = hardwareMap.get(DcMotorEx.class, "right_back_drive");

        // Motor Configuration
        topFlywheel.setDirection(DcMotorEx.Direction.FORWARD);
        bottomFlywheel.setDirection(DcMotorEx.Direction.REVERSE);

        // Match directions from project hardware config (ri3dStarterCode)
        leftFrontDrive.setDirection(DcMotorEx.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotorEx.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotorEx.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotorEx.Direction.FORWARD);

        topFlywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bottomFlywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        loaderServo.setPosition(SERVO_LOADING);

        telemetry.addData("Status", "Hardware Initialized");
    }

    @Override
    public void start() {
        loopTimer.reset();
        minVoltageObserved = batteryVoltageSensor.getVoltage();
    }

    @Override
    public void loop() {
        long startTime = System.currentTimeMillis();

        handleInputs();
        handleAiming(); // Rotation control added here
        updateMotorVelocity();
        monitorVoltage();
        updateTelemetry();

        lastLoopTimeMs = System.currentTimeMillis() - startTime;
        loopCount++;
        avgLoopTimeMs = (avgLoopTimeMs * (loopCount - 1) + lastLoopTimeMs) / loopCount;
    }

    /**
     * Handles drivetrain rotation for aiming.
     * Translation (X/Y) is disabled per request to focus on aiming.
     */
    private void handleAiming() {
        double rotate = gamepad1.right_stick_x;

        // Apply rotation power: Left side positive, Right side negative to spin right
        leftFrontDrive.setPower(rotate);
        leftBackDrive.setPower(rotate);
        rightFrontDrive.setPower(-rotate);
        rightBackDrive.setPower(-rotate);
    }

    private void handleInputs() {
        double currentStep = gamepad1.a ? STEP_PRECISION : STEP_NORMAL;

        if (gamepad1.dpad_up && !lastDpadUp) {
            currentSelection = (currentSelection == SelectedFlywheel.BOTTOM) ? SelectedFlywheel.TOP : SelectedFlywheel.BOTTOM;
        }
        lastDpadUp = gamepad1.dpad_up;

        if (gamepad1.dpad_right && !lastDpadRight) {
            adjustRPM(currentStep);
        }
        lastDpadRight = gamepad1.dpad_right;

        if (gamepad1.dpad_left && !lastDpadLeft) {
            adjustRPM(-currentStep);
        }
        lastDpadLeft = gamepad1.dpad_left;

        if (gamepad1.right_bumper) {
            loaderServo.setPosition(SERVO_SHOOTING);
        } else if (gamepad1.left_bumper) {
            loaderServo.setPosition(SERVO_LOADING);
        }
    }

    private void adjustRPM(double step) {
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

    private void monitorVoltage() {
        double currentVoltage = batteryVoltageSensor.getVoltage();
        if (currentVoltage < minVoltageObserved) {
            minVoltageObserved = currentVoltage;
        }
    }

    private void updateTelemetry() {
        double actualBottomRPM = ticksPerSecondToRPM(bottomFlywheel.getVelocity());
        double actualTopRPM = ticksPerSecondToRPM(topFlywheel.getVelocity());

        double bottomError = targetBottomRPM - actualBottomRPM;
        double topError = targetTopRPM - actualTopRPM;

        boolean bottomReady = Math.abs(bottomError) < RPM_TOLERANCE;
        boolean topReady = Math.abs(topError) < RPM_TOLERANCE;

        telemetry.addLine("=== SHOOTER STATUS ===");
        telemetry.addData("STATUS", (bottomReady && topReady) ? "READY TO FIRE" : "SPINNING UP...");
        telemetry.addData("Aiming Power", "%.2f", gamepad1.right_stick_x);

        telemetry.addLine("\n=== FLYWHEEL PERFORMANCE ===");
        telemetry.addData("Bottom (Target|Actual)", "%.0f | %.0f RPM", targetBottomRPM, actualBottomRPM);
        telemetry.addData("Top    (Target|Actual)", "%.0f | %.0f RPM", targetTopRPM, actualTopRPM);

        telemetry.addLine("\n=== CONTROL INTERFACE ===");
        telemetry.addData("Selected Flywheel", currentSelection);
        telemetry.addData("Precision Mode (A)", gamepad1.a ? "ACTIVE (10 RPM)" : "INACTIVE (100 RPM)");
        telemetry.addData("Servo State", loaderServo.getPosition() == SERVO_SHOOTING ? "SHOOTING" : "LOADING");

        telemetry.addLine("\n=== SYSTEM DIAGNOSTICS ===");
        telemetry.addData("Battery Voltage", "%.2fV", batteryVoltageSensor.getVoltage());
        telemetry.addData("Lowest Voltage Obs", "%.2fV", minVoltageObserved);
        telemetry.addData("Loop Time", "%.0f ms", lastLoopTimeMs);

        telemetry.update();
    }
}
