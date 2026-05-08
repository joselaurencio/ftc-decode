package org.firstinspires.ftc.teamcode.opmodes;

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

import java.util.ArrayList;

@TeleOp(name = "CALIBRATION MODE", group = "TEST")
public class CalibrationMode extends OpMode {

    // ===== DRIVETRAIN =====
    private DcMotor leftFront, rightFront, leftBack, rightBack;

    // ===== SHOOTER =====
    private DcMotorEx leftLauncher, rightLauncher;
    private CRServo   leftFeeder,   rightFeeder;
    private Servo     diverter;

    // ===== INTAKE =====
    // intake1 and intake2 face opposite directions physically.
    // Both are set REVERSE here; setIntakePower() applies opposite
    // signs so the rollers pull inward together.
    private DcMotorEx intake1;   // encoder-controlled
    private DcMotorEx intake2;   // encoder-controlled, spins opposite to intake1

    // ===== VISION =====
    private LimelightVision limelight;

    // ===== SHOOTER CONSTANTS =====
    private static final double RPM_STEP      = 10;
    private static final double RPM_MIN       = 500;
    private static final double RPM_MAX       = 6000;
    private static final double PIDF_P        = 300;
    private static final double PIDF_F        = 10;
    private static final double FEED_TIME_SEC = 2008;
    private static final double FULL_SPEED    = 1.0;
    private static final double STOP_SPEED    = 0.0;
    private static final double DIVERTER_LEFT  = .4;
    private static final double DIVERTER_RIGHT = 0.0;

    private double currentRPM             = 4000;
    private double launcherTargetVelocity = currentRPM * 28.0 / 60.0;

    // ===== INTAKE RPM =====
    // Tune INTAKE_RPM to whatever speed pulls game elements cleanly.
    // INTAKE_TICKS_PER_REV should match your motor — default is 28
    // (bare REV HD Hex / Core Hex shaft encoder). Change if different.
    // setIntakePower() converts RPM → ticks/sec and calls setVelocity(),
    // so the intake holds speed under load just like the launchers.
    private static final double INTAKE_TICKS_PER_REV = 146.44;
    private static final double INTAKE_PIDF_P        = 10.0;  // tune if intake hunts
    private static final double INTAKE_PIDF_F        = 1.0;   // tune for feedforward
    private double intakeRPM             = 2000;  // target RPM — adjust as needed
    private double intakeTargetVelocity  = intakeRPM * INTAKE_TICKS_PER_REV / 60.0;

    // ===== FEEDER TIMER =====
    private ElapsedTime leftFeederTimer  = new ElapsedTime();
    private ElapsedTime rightFeederTimer = new ElapsedTime();
    private boolean leftFeeding  = false;
    private boolean rightFeeding = false;

    // ===== SHOOTER STATE =====
    private boolean flywheelsRunning = false;
    private boolean diverterLeft     = true;

    // ===== INTAKE STATE =====
    // ON      = both motors rolling inward at intakeRPM
    // OFF     = both motors stopped
    // REVERSE = both motors rolling outward at intakeRPM (eject)
    private enum IntakeState { ON, OFF, REVERSE }
    private IntakeState intakeState = IntakeState.OFF;

    // ===== DATA STORAGE =====
    ArrayList<Double> baseOffsets  = new ArrayList<>();
    ArrayList<Double> leftOffsets  = new ArrayList<>();
    ArrayList<Double> rightOffsets = new ArrayList<>();
    ArrayList<Double> distances    = new ArrayList<>();

    // ===== SHOT SNAPSHOT LOGS =====
    ArrayList<Double> shotTxLog       = new ArrayList<>();
    ArrayList<Double> shotDistLog     = new ArrayList<>();
    ArrayList<Double> shotLeftRpmLog  = new ArrayList<>();
    ArrayList<Double> shotRightRpmLog = new ArrayList<>();
    ArrayList<String> shotSideLog     = new ArrayList<>();

    // ===== BUTTON EDGE DETECTION =====
    private boolean lastA         = false;
    private boolean lastB         = false;
    private boolean lastX         = false;
    private boolean lastY         = false;
    private boolean lastLB        = false;
    private boolean lastRB        = false;
    private boolean lastDpadDown  = false;
    private boolean lastDpadUp    = false;
    private boolean lastDpadLeft  = false;
    private boolean lastDpadRight = false;

    // =========================================================
    // INIT
    // =========================================================
    @Override
    public void init() {

        // ── Drivetrain ──
        leftFront  = hardwareMap.get(DcMotor.class, "left_front_drive");
        rightFront = hardwareMap.get(DcMotor.class, "right_front_drive");
        leftBack   = hardwareMap.get(DcMotor.class, "left_back_drive");
        rightBack  = hardwareMap.get(DcMotor.class, "right_back_drive");

        leftFront.setDirection(DcMotorSimple.Direction.FORWARD);
        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setDirection(DcMotorSimple.Direction.FORWARD);
        rightBack.setDirection(DcMotorSimple.Direction.REVERSE);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // ── Shooters ──
        leftLauncher  = hardwareMap.get(DcMotorEx.class, "left_launcher");
        rightLauncher = hardwareMap.get(DcMotorEx.class, "right_launcher");

        leftLauncher.setDirection(DcMotorSimple.Direction.FORWARD);
        rightLauncher.setDirection(DcMotorSimple.Direction.FORWARD);

        leftLauncher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightLauncher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftLauncher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightLauncher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftLauncher.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,
                new PIDFCoefficients(PIDF_P, 0, 0, PIDF_F));
        rightLauncher.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,
                new PIDFCoefficients(PIDF_P, 0, 0, PIDF_F));

        // ── Feeders ──
        leftFeeder  = hardwareMap.get(CRServo.class, "left_feeder");
        rightFeeder = hardwareMap.get(CRServo.class, "right_feeder");

        leftFeeder.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFeeder.setDirection(DcMotorSimple.Direction.FORWARD);

        leftFeeder.setPower(STOP_SPEED);
        rightFeeder.setPower(STOP_SPEED);

        // ── Diverter ──
        diverter = hardwareMap.get(Servo.class, "diverter");
        diverter.setPosition(DIVERTER_LEFT);

        // ── Intake ──
        intake1 = hardwareMap.get(DcMotorEx.class, "intake1");
        intake2 = hardwareMap.get(DcMotorEx.class, "intake2");

        // Your original directions — both REVERSE in hardware config;
        // setIntakePower() flips the sign on intake2 so they roll inward together.
        intake1.setDirection(DcMotorSimple.Direction.REVERSE);
        intake2.setDirection(DcMotorSimple.Direction.FORWARD);

        intake1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intake2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        intake1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        intake1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,
                new PIDFCoefficients(INTAKE_PIDF_P, 0, 0, INTAKE_PIDF_F));
        intake2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,
                new PIDFCoefficients(INTAKE_PIDF_P, 0, 0, INTAKE_PIDF_F));

        setIntakeVelocity(0);

        // ── Vision ──
        limelight = new LimelightVision(hardwareMap);

        telemetry.addLine("CALIBRATION MODE Ready");
        telemetry.addLine("See CONTROLS section below");
        telemetry.update();
    }

    // =========================================================
    // MAIN LOOP
    // =========================================================
    @Override
    public void loop() {

        limelight.update();

        // ── Read vision ──
        double tx       = 0;
        double distance = 0;
        if (limelight.hasTarget()) {
            tx       = limelight.getTx();
            distance = limelight.getDistanceFromArea();
        }

        // ── Read buttons ──
        boolean a         = gamepad1.a;
        boolean b         = gamepad1.b;
        boolean x         = gamepad1.x;
        boolean y         = gamepad1.y;
        boolean lb        = gamepad1.left_bumper;
        boolean rb        = gamepad1.right_bumper;
        boolean dpadDown  = gamepad1.dpad_down;
        boolean dpadUp    = gamepad1.dpad_up;
        boolean dpadLeft  = gamepad1.dpad_left;
        boolean dpadRight = gamepad1.dpad_right;

        // =========================================================
        // DRIVETRAIN
        // =========================================================
        double forward = -gamepad1.left_stick_y;
        double strafe  =  gamepad1.left_stick_x;
        double rotate  =  gamepad1.right_stick_x;
        mecanumDrive(forward, strafe, rotate);

        // =========================================================
        // RPM ADJUSTMENT  —  D-Pad Left = -10, D-Pad Right = +10
        // =========================================================
        if (dpadLeft && !lastDpadLeft) {
            currentRPM = Math.max(RPM_MIN, currentRPM - RPM_STEP);
            launcherTargetVelocity = currentRPM * 28.0 / 60.0;
        }
        if (dpadRight && !lastDpadRight) {
            currentRPM = Math.min(RPM_MAX, currentRPM + RPM_STEP);
            launcherTargetVelocity = currentRPM * 28.0 / 60.0;
        }

        // =========================================================
        // FLYWHEEL  —  Left Trigger = start, Right Trigger = stop
        // =========================================================
        if (gamepad1.left_trigger > 0.5 && !flywheelsRunning) {
            flywheelsRunning = true;
        }
        if (gamepad1.right_trigger > 0.5) {
            flywheelsRunning = false;
            leftLauncher.setVelocity(0);
            rightLauncher.setVelocity(0);
        }
        if (flywheelsRunning) {
            leftLauncher.setVelocity(launcherTargetVelocity);
            rightLauncher.setVelocity(launcherTargetVelocity);
        }

        // =========================================================
        // FIRE LEFT FEEDER  —  Left Bumper
        // =========================================================
        if (lb && !lastLB) {
            leftFeeding = true;
            leftFeeder.setPower(FULL_SPEED);
            leftFeederTimer.reset();
            if (limelight.hasTarget()) {
                shotTxLog.add(tx);
                shotDistLog.add(distance);
                shotLeftRpmLog.add(leftLauncher.getVelocity() * 60.0 / 28.0);
                shotRightRpmLog.add(rightLauncher.getVelocity() * 60.0 / 28.0);
                shotSideLog.add("LEFT");
            }
        }
        if (leftFeeding && leftFeederTimer.seconds() > FEED_TIME_SEC) {
            leftFeeding = false;
            leftFeeder.setPower(STOP_SPEED);
        }

        // =========================================================
        // FIRE RIGHT FEEDER  —  Right Bumper
        // =========================================================
        if (rb && !lastRB) {
            rightFeeding = true;
            rightFeeder.setPower(FULL_SPEED);
            rightFeederTimer.reset();
            if (limelight.hasTarget()) {
                shotTxLog.add(tx);
                shotDistLog.add(distance);
                shotLeftRpmLog.add(leftLauncher.getVelocity() * 60.0 / 28.0);
                shotRightRpmLog.add(rightLauncher.getVelocity() * 60.0 / 28.0);
                shotSideLog.add("RIGHT");
            }
        }
        if (rightFeeding && rightFeederTimer.seconds() > FEED_TIME_SEC) {
            rightFeeding = false;
            rightFeeder.setPower(STOP_SPEED);
        }

        // =========================================================
        // DIVERTER TOGGLE  —  D-Pad Down
        // =========================================================
        if (dpadDown && !lastDpadDown) {
            diverterLeft = !diverterLeft;
            diverter.setPosition(diverterLeft ? DIVERTER_LEFT : DIVERTER_RIGHT);
        }

        // =========================================================
        // INTAKE
        // D-Pad Up          = toggle intake IN / OFF
        // Both Bumpers held = REVERSE / eject
        // =========================================================
        if (lb && rb) {
            setIntakeVelocity(-intakeTargetVelocity);
        } else if (dpadUp && !lastDpadUp) {
            if (intakeState == IntakeState.ON) {
                intakeState = IntakeState.OFF;
                setIntakeVelocity(0);
            } else {
                intakeState = IntakeState.ON;
                setIntakeVelocity(intakeTargetVelocity);
            }
        } else if (!lb && !rb) {
            switch (intakeState) {
                case ON:      setIntakeVelocity( intakeTargetVelocity); break;
                case REVERSE: setIntakeVelocity(-intakeTargetVelocity); break;
                case OFF:     setIntakeVelocity(0);                     break;
            }
        }

        // =========================================================
        // CALIBRATION DATA RECORDING
        // =========================================================
        if (a && !lastA && limelight.hasTarget()) {
            baseOffsets.add(tx);
            distances.add(distance);
        }
        if (b && !lastB && limelight.hasTarget()) {
            leftOffsets.add(tx);
        }
        if (x && !lastX && limelight.hasTarget()) {
            rightOffsets.add(tx);
        }
        if (y && !lastY) {
            baseOffsets.clear();
            leftOffsets.clear();
            rightOffsets.clear();
            distances.clear();
            shotTxLog.clear();
            shotDistLog.clear();
            shotLeftRpmLog.clear();
            shotRightRpmLog.clear();
            shotSideLog.clear();
        }

        // ── Edge detection update ──
        lastA         = a;
        lastB         = b;
        lastX         = x;
        lastY         = y;
        lastLB        = lb;
        lastRB        = rb;
        lastDpadDown  = dpadDown;
        lastDpadUp    = dpadUp;
        lastDpadLeft  = dpadLeft;
        lastDpadRight = dpadRight;

        // =========================================================
        // CALCULATIONS
        // =========================================================
        double avgBase  = average(baseOffsets);
        double avgLeft  = average(leftOffsets);
        double avgRight = average(rightOffsets);

        double suggestedLeftOffset  = avgBase - avgLeft;
        double suggestedRightOffset = avgRight - avgBase;

        double leftRPM   = leftLauncher.getVelocity()  * 60.0 / 28.0;
        double rightRPM  = rightLauncher.getVelocity() * 60.0 / 28.0;
        double intake1RPM = intake1.getVelocity() * 60.0 / INTAKE_TICKS_PER_REV;
        double intake2RPM = intake2.getVelocity() * 60.0 / INTAKE_TICKS_PER_REV;

        // =========================================================
        // TELEMETRY
        // =========================================================
        telemetry.addLine("======= LIVE DATA =======");
        telemetry.addData("tx (degrees)",   String.format("%.3f", tx));
        telemetry.addData("distance (cm)",  String.format("%.1f", distance));
        telemetry.addData("Has Target",     limelight.hasTarget() ? "YES" : "NO");
        telemetry.addData("Left RPM",       String.format("%.0f", leftRPM));
        telemetry.addData("Right RPM",      String.format("%.0f", rightRPM));
        telemetry.addData("RPM Target",     String.format("%.0f", currentRPM));
        telemetry.addData("Intake1 RPM",    String.format("%.0f", intake1RPM));
        telemetry.addData("Intake2 RPM",    String.format("%.0f", intake2RPM));
        telemetry.addData("Intake RPM Tgt", String.format("%.0f", intakeRPM));

        telemetry.addLine("");
        telemetry.addLine("======= STATES =======");
        telemetry.addData("Flywheels", flywheelsRunning ? "RUNNING" : "OFF");
        telemetry.addData("Diverter",  diverterLeft     ? "LEFT"    : "RIGHT");
        telemetry.addData("Intake",    intakeState);
        telemetry.addData("L Feeder",  leftFeeding      ? "FEEDING" : "idle");
        telemetry.addData("R Feeder",  rightFeeding     ? "FEEDING" : "idle");

        telemetry.addLine("");
        telemetry.addLine("======= RECORDED SAMPLES =======");
        telemetry.addData("Base Samples",  baseOffsets.size());
        telemetry.addData("Left Samples",  leftOffsets.size());
        telemetry.addData("Right Samples", rightOffsets.size());

        telemetry.addLine("");
        telemetry.addLine("======= RESULTS =======");
        telemetry.addData("BASE_OFFSET",          String.format("%.4f", avgBase));
        telemetry.addData("LEFT_SHOOTER_OFFSET",  String.format("%.4f", suggestedLeftOffset));
        telemetry.addData("RIGHT_SHOOTER_OFFSET", String.format("%.4f", suggestedRightOffset));

        telemetry.addLine("");
        telemetry.addLine("======= SHOT LOG (last 5) =======");
        telemetry.addData("Total Shots", shotTxLog.size());
        int logSize  = shotTxLog.size();
        int showFrom = Math.max(0, logSize - 5);
        for (int i = showFrom; i < logSize; i++) {
            telemetry.addLine(String.format(
                    "#%d [%s] tx=%.3f° dist=%.1fcm L=%.0f R=%.0f RPM",
                    i + 1,
                    shotSideLog.get(i),
                    shotTxLog.get(i),
                    shotDistLog.get(i),
                    shotLeftRpmLog.get(i),
                    shotRightRpmLog.get(i)
            ));
        }
        if (logSize == 0) telemetry.addLine("  No shots fired yet");

        telemetry.addLine("");
        telemetry.addLine("======= CONTROLS =======");
        telemetry.addLine("Left Stick       = Drive");
        telemetry.addLine("Right Stick      = Rotate");
        telemetry.addLine("Left Trigger     = Start Flywheels");
        telemetry.addLine("Right Trigger    = Stop Flywheels");
        telemetry.addLine("D-Pad LEFT       = RPM - 10");
        telemetry.addLine("D-Pad RIGHT      = RPM + 10");
        telemetry.addLine("D-Pad Up         = Toggle Intake IN/OFF");
        telemetry.addLine("D-Pad Down       = Toggle Diverter");
        telemetry.addLine("Left Bumper      = Fire LEFT feeder + log shot");
        telemetry.addLine("Right Bumper     = Fire RIGHT feeder + log shot");
        telemetry.addLine("Both Bumpers     = Intake REVERSE (eject)");
        telemetry.addLine("A                = Save BASE offset");
        telemetry.addLine("B                = Save LEFT offset");
        telemetry.addLine("X                = Save RIGHT offset");
        telemetry.addLine("Y                = Clear ALL data + shot log");

        telemetry.update();
    }

    // =========================================================
    // INTAKE HELPER
    // =========================================================
    // Drives both intake motors at the target velocity in opposite
    // directions so their rollers pull inward together.
    //
    //   velocity > 0 → intake IN
    //   velocity < 0 → intake OUT (eject)
    //   velocity = 0 → stop both
    //
    // intake2 receives the negated value because it faces the
    // opposite direction physically.
    // =========================================================
    private void setIntakeVelocity(double velocity) {
        intake1.setVelocity( velocity);
        intake2.setVelocity(-velocity);
    }

    // =========================================================
    // MECANUM DRIVE
    // =========================================================
    void mecanumDrive(double forward, double strafe, double rotate) {
        double denominator = Math.max(Math.abs(forward) + Math.abs(strafe) + Math.abs(rotate), 1);

        leftFront.setPower((forward + strafe + rotate) / denominator);
        rightFront.setPower((forward - strafe - rotate) / denominator);
        leftBack.setPower((forward - strafe + rotate) / denominator);
        rightBack.setPower((forward + strafe - rotate) / denominator);
    }

    // =========================================================
    // AVERAGE HELPER
    // =========================================================
    double average(ArrayList<Double> list) {
        if (list.size() == 0) return 0;
        double sum = 0;
        for (double v : list) sum += v;
        return sum / list.size();
    }

    // =========================================================
    // STOP
    // =========================================================
    @Override
    public void stop() {
        leftLauncher.setVelocity(0);
        rightLauncher.setVelocity(0);
        leftFeeder.setPower(STOP_SPEED);
        rightFeeder.setPower(STOP_SPEED);
        setIntakeVelocity(0);
    }
}