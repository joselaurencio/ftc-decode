package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "RGB Indicator DPad Test", group = "Test")
public class RGBIndicatorTest extends OpMode {

    private Servo rgb;
    private double position = 0.0;

    // Button state tracking (for debounce)
    private boolean dpadUpPrev = false;
    private boolean dpadDownPrev = false;

    @Override
    public void init() {
        rgb = hardwareMap.get(Servo.class, "rgb_indicator");
        telemetry.addLine("RGB Indicator Initialized");
    }

    @Override
    public void loop() {

        // D-pad UP → increase
        if (gamepad1.dpad_up && !dpadUpPrev) {
            position += 0.01;
        }

        // D-pad DOWN → decrease
        if (gamepad1.dpad_down && !dpadDownPrev) {
            position -= 0.01;
        }

        // Clamp between 0 and 1
        position = Math.max(0.0, Math.min(1.0, position));

        // Set servo
        rgb.setPosition(position);

        // Save previous states
        dpadUpPrev = gamepad1.dpad_up;
        dpadDownPrev = gamepad1.dpad_down;

        // Telemetry
        telemetry.addData("Position", "%.2f", position);
        telemetry.addLine("Use D-Pad Up/Down (0.05 increments)");
        telemetry.update();
    }
}