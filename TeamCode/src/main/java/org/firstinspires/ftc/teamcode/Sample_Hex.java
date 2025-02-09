package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="Motor_test")
public class Sample_Hex extends OpMode {
    private DcMotor hex_motor;
    private Servo servo;

    @Override
    public void init() {
        hex_motor = hardwareMap.get(DcMotor.class, "hex_motor");
        servo = hardwareMap.get(Servo.class, "servo");

        // Set initial servo position
        servo.setPosition(0.5);
    }

    @Override
    public void loop() {
        // Control for hex motor
        double hex_up = -gamepad1.right_stick_y;
        hex_motor.setPower(Range.clip(hex_up, -1.0, 1.0));

        // Control for servo using dpad
        double currentPosition = servo.getPosition();
        double increment = 0.01;

        if (gamepad1.dpad_up) {
            servo.setPosition(Range.clip(currentPosition + increment, 0.0, 1.0));
        } else if (gamepad1.dpad_down) {
            servo.setPosition(Range.clip(currentPosition - increment, 0.0, 1.0));
        }

        // Telemetry for debugging
        telemetry.addData("Hex Motor Power", hex_up);
        telemetry.addData("Servo Position", servo.getPosition());
        telemetry.update();
    }
}
